extern crate sdl2;

use sdl2::pixels::Color;
use sdl2::rect::Rect;
use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use std::time::Duration;

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

    vram: [u8; 0x2000],     // Video RAM
    oam: [u8; 0xA0],        // Object Attribute Memory
    color_offset: f32,
    scale_factor: u32,
}

impl PPU {
    pub fn new() -> PPU {
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
            vram: [0; 0x2000],
            oam: [0; 0xA0],
            color_offset: 0.0,
            scale_factor: 4
        }
    }

    /// Update PPU state based on the number of cycles
    pub fn update(&mut self, cycles: u32) {
        self.color_offset = (self.color_offset + 0.005) % 1.0;
    }

    pub fn render(&mut self, canvas: &mut sdl2::render::Canvas<sdl2::video::Window>) {
        let color = self.hsv_to_rgb(self.color_offset, 1.0, 1.0);

        // Render the entire screen with the calculated color
        canvas.set_draw_color(color);
        let rect = Rect::new(0, 0, 160 * self.scale_factor, 144 * self.scale_factor);
        canvas.fill_rect(rect).unwrap();
    }

    fn hsv_to_rgb(&self, h: f32, s: f32, v: f32) -> Color {
        let i = (h * 6.0).floor() as i32;
        let f = h * 6.0 - i as f32;
        let p = v * (1.0 - s);
        let q = v * (1.0 - f * s);
        let t = v * (1.0 - (1.0 - f) * s);

        let (r, g, b) = match i % 6 {
            0 => (v, t, p),
            1 => (q, v, p),
            2 => (p, v, t),
            3 => (p, q, v),
            4 => (t, p, v),
            5 => (v, p, q),
            _ => (0.0, 0.0, 0.0),
        };

        Color::RGB((r * 255.0) as u8, (g * 255.0) as u8, (b * 255.0) as u8)
    }

    /// Handle LCD Control based on the LCDC Register
    fn handle_lcdc(&mut self) {

    }
}