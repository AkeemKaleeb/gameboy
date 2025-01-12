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
            bus,
            scale_factor,
        }
    }

    /// Update PPU state based on the number of cycles
    pub fn update(&mut self, _cycles: u32) {
        
    }

    /// Call to render frame
    pub fn render(&mut self, canvas: &mut Canvas<Window>) {
        self.render_background(canvas);
    }

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

    /// Handle LCD Control based on the LCDC Register
    fn handle_lcdc(&mut self) {

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