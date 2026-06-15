unsafe extern "C" {
    fn abort() -> !;
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    unsafe { abort() }
}
