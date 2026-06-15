use core::alloc::{GlobalAlloc, Layout};

#[global_allocator]
static HEAP: Esp32Alloc = Esp32Alloc;

/*
#[alloc_error_handler]
#[allow(dead_code)]
fn on_oom(_layout: Layout) -> ! {
    unsafe { abort() }
}
*/

unsafe extern "C" {
    fn heap_caps_malloc(size: usize, caps: u32) -> *mut u8;
    fn heap_caps_realloc(ptr: *mut u8, size: usize, caps: u32) -> *mut u8;
    fn heap_caps_free(ptr: *mut u8);
}

const MALLOC_CAP_8BIT: u32 = 1u32 << 2;

struct Esp32Alloc;

unsafe impl Sync for Esp32Alloc {}

unsafe impl GlobalAlloc for Esp32Alloc {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        unsafe {
            heap_caps_malloc(layout.size(), MALLOC_CAP_8BIT as _) as *mut _
        }
    }

    unsafe fn realloc(&self, ptr: *mut u8, _layout: Layout, new_size: usize) -> *mut u8 {
        unsafe {
            heap_caps_realloc(ptr as *mut _, new_size, MALLOC_CAP_8BIT as _) as *mut _
        }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, _layout: Layout) {
        unsafe {
            heap_caps_free(ptr as *mut _);
        }
    }
}
