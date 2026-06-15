use core::fmt::Write;

unsafe extern "C" {
    fn write(fd: i32, buf: *const u8, count: usize) -> isize;
}

pub struct Stdout;

pub fn stdout() -> Stdout {
    Stdout
}

impl Write for Stdout {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let result = unsafe { write(0, s.as_ptr(), s.len()) };

        // Nobody actually wants to output a string more than 2 GB in length on an ESP32.
        if result != s.len() as isize {
            return Err(core::fmt::Error)
        }

        Ok(())
    }
}
