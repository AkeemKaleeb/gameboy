extern crate sdl2;
use crate::bus::Bus;

use sdl2::pixels::Color;
use sdl2::rect::Rect;
use sdl2::render::Canvas;
use sdl2::video::Window;
use std::cell::RefCell;
use std::rc::Rc;

pub struct PPU {
    lcdc: u8,               // LCD Control
    stat: u8,               // LCDC Status
    scy: u8,                // Scroll Y
    scx: u8,                // Scroll X
    ly: u8,                 // Current Scanline
    lyc: u8,                // LY Compare
    bgp: u8,                // BG Palette Data
    obp0: u8,               // Object Palette 0 Data
    obp1: u8,               // Object Palette 1 Data
    wy: u8,                 // Window Y Position
    wx: u8,                 // Window X Position
    mode: u8,               // Current Mode
    mode_clock: u32,        // Mode Clock

    bus: Rc<RefCell<Bus>>,
    scale_factor: u32,
}

impl PPU {
    pub fn new(bus: Rc<RefCell<Bus>>, scale_factor: u32) -> PPU {
        PPU {
            lcdc: 0x91,
            stat: 0x85,
            scy: 0x00,
            scx: 0x00,
            ly: 0x00,
            lyc: 0x00,
            bgp: 0xFC,
            obp0: 0xFF,
            obp1: 0xFF,
            wy: 0x00,
            wx: 0x00,
            mode: 2,
            mode_clock: 0,
            bus,
            scale_factor,
        }
    }

    /// Update PPU state based on the number of cycles
    pub fn update(&mut self, cycles: u32, canvas: &mut Canvas<Window>) {
        self.mode_clock += cycles;
        
        match self.mode {
            0 => {  // HBlank
                if self.mode_clock >= 204 {
                    self.mode_clock = 0;
                    self.ly += 1;

                    if self.ly == 144 {
                        self.mode = 1;
                        self.enter_vblank();
                    } else {
                        self.mode = 2;
                    }
                }
            },
            1 => {  // VBlank
                if self.mode_clock >= 456 {
                    self.mode_clock = 0;
                    self.ly += 1;

                    if self.ly > 153 {
                        self.ly = 0;
                        self.mode = 2;   
                    }
                }
            },
            2 => {  // OAM Search
                if self.mode_clock >= 80 {
                    self.mode_clock = 0;
                    self.mode = 3;
                }
            },
            3 => {  // Pixel Transfer
                if self.mode_clock >= 172 {
                    self.mode_clock = 0;
                    self.mode = 0;
                    self.render(canvas);
                }
            },
            _ => {  // Should never happen
                self.mode = 0;
                self.mode_clock = 0;
            }
        }

        self.handle_stat();
    }

    /// Call to render frame
    fn render(&mut self, canvas: &mut Canvas<Window>) {
        self.render_background(canvas);
        self.render_window(canvas);
        self.render_sprites(canvas);
    }

    /// Enter HBlank state 
    fn enter_hblank(&mut self) {
        // Set the HBlank flag in the STAT register
        self.stat &= 0xFE;
    }

    /// Enter VBlank state
    fn enter_vblank(&mut self) {
        self.stat |= 0x01;                                                  // Set the VBlank flag in the STAT register
        self.bus.borrow_mut().write_byte(0xFF0F, 0x01);     // Request VBlank interrupt
    }

    /// Render background layer
    fn render_background(&mut self, canvas: &mut Canvas<Window>) {
        let tile_map_base = if self.lcdc & 0x08 != 0 { 0x9C00 } else { 0x9800 };
        let tile_data_base = if self.lcdc & 0x10 != 0 { 0x8000 } else { 0x8800 };

        for y in 0..144 {
            for x in 0..160 {
                let tile_x = (x + self.scx as u32) / 8;
                let tile_y = (y + self.scy as u32) / 8;
                let tile_index = self.bus.borrow().read_byte((tile_map_base + tile_y * 32 + tile_x) as u16);
                let tile_address = tile_data_base + (tile_index as u16 * 16);
                let pixel_x = (x + self.scx as u32) % 8;
                let pixel_y = (y + self.scy as u32) % 8;
                let byte1 = self.bus.borrow().read_byte((tile_address + (pixel_y as u16) * 2) as u16);
                let byte2 = self.bus.borrow().read_byte((tile_address + (pixel_y as u16) * 2 + 1) as u16);
                let bit = 7 - pixel_x;
                let color_id = ((byte1 >> bit) & 0x01) | (((byte2 >> bit) & 0x01) << 1);
                let color = self.get_color(color_id, self.bgp); 

                let rect = Rect::new(
                    (x * self.scale_factor) as i32,
                    (y * self.scale_factor) as i32,
                    self.scale_factor,
                    self.scale_factor,
                );

                canvas.set_draw_color(color);
                canvas.fill_rect(rect).unwrap();
            }
        }
    }

    /// Render Mid-ground Sprites Layer
    fn render_sprites(&mut self, canvas: &mut Canvas<Window>) {
        if self.lcdc & 0x02 == 0 {
            return;
        }

        for i in 0..40 {
            let sprite_base = 0xFE00 + i * 4;
            let y = self.bus.borrow().read_byte(sprite_base) as i32 - 16;
            let x = self.bus.borrow().read_byte(sprite_base + 1) as i32 - 8;
            let tile_index = self.bus.borrow().read_byte(sprite_base + 2);
            let attributes = self.bus.borrow().read_byte(sprite_base + 3);

            let tile_address = 0x8000 + (tile_index as u16 * 16);
            let flip_x = attributes & 0x20 != 0;
            let flip_y = attributes & 0x40 != 0;
            let palette = if attributes & 0x10 != 0 { self.obp1 } else { self.obp0 };

            for py in 0..8 {
                for px in 0..8 {
                    let pixel_x = if flip_x { 7 - px } else { px };
                    let pixel_y = if flip_y { 7 - py } else { py };
                    let byte1 = self.bus.borrow().read_byte((tile_address + (pixel_y as u16) * 2) as u16);
                    let byte2 = self.bus.borrow().read_byte((tile_address + (pixel_y as u16) * 2 + 1) as u16);
                    let bit = 7 - pixel_x;
                    let color_id = ((byte1 >> bit) & 0x01) | (((byte2 >> bit) & 0x01) << 1);
                    let color = self.get_color(color_id, palette);

                    let rect = Rect::new(
                        (x + px as i32) * self.scale_factor as i32,
                        (y + py as i32) * self.scale_factor as i32,
                        self.scale_factor,
                        self.scale_factor,
                    );

                    canvas.set_draw_color(color);
                    canvas.fill_rect(rect).unwrap();
                }
            }
        }
    }

    /// Render Foreground Window Layer
    fn render_window(&mut self, canvas: &mut Canvas<Window>) {
        if self.lcdc & 0x20 == 0 {
            return;
        }

        let tile_map_base = if self.lcdc & 0x40 != 0 { 0x9C00 } else { 0x9800 };
        let tile_data_base = if self.lcdc & 0x10 != 0 { 0x8000 } else { 0x8800 };

        for y in 0..144 {
            for x in 0..160 {
                if x < self.wx as u32 || y < self.wy as u32 {
                    continue;
                }

                let tile_x = (x - self.wx as u32) / 8;
                let tile_y = (y - self.wy as u32) / 8;
                let tile_index = self.bus.borrow().read_byte((tile_map_base + tile_y * 32 + tile_x) as u16);
                let tile_address = tile_data_base + (tile_index as u16 * 16);
                let pixel_x = (x - self.wx as u32) % 8;
                let pixel_y = (y - self.wy as u32) % 8;
                let byte1 = self.bus.borrow().read_byte((tile_address + (pixel_y as u16) * 2) as u16);
                let byte2 = self.bus.borrow().read_byte((tile_address + (pixel_y as u16) * 2 + 1) as u16);
                let bit = 7 - pixel_x;
                let color_id = ((byte1 >> bit) & 0x01) | (((byte2 >> bit) & 0x01) << 1);
                let color = self.get_color(color_id, self.bgp); 

                let rect = Rect::new(
                    (x * self.scale_factor) as i32,
                    (y * self.scale_factor) as i32,
                    self.scale_factor,
                    self.scale_factor,
                );

                canvas.set_draw_color(color);
                canvas.fill_rect(rect).unwrap();
            }
        }
    }

    /// Handle LCD Control based on the LCDC Register
    fn handle_lcdc(&mut self) {
        self.lcdc = self.bus.borrow().read_byte(0xFF40);
    }

    /// Handle LCD Status based on STAT register
    fn handle_stat(&mut self) {
        self.stat = self.bus.borrow().read_byte(0xFF41);

        // Request Interrupts if enabled
        if (self.stat & 0x20 != 0) && (self.ly == self.lyc)
    }

    fn get_color(&self, color_id: u8, palette: u8) -> Color {
        match (palette >> (color_id * 2)) & 0x03 {
            0 => Color::RGB(255, 255, 255),
            1 => Color::RGB(192, 192, 192),
            2 => Color::RGB(96, 96, 96),
            3 => Color::RGB(0, 0, 0),
            _ => Color::RGB(255, 255, 255),
        }
    }
}