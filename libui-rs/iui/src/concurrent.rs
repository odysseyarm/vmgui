use callback_helpers::{from_void_ptr, to_heap_ptr};
use std::os::raw::c_void;
use std::sync::Arc;
use std::future::Future;
use std::marker::PhantomData;

/// Evidence that it's safe to call `ui_sys:uiQueueMain`; ie that `UI:init()`
/// has been run.
#[derive(Copy,Clone)]
pub struct Context {
    pub(crate) _pd: PhantomData<()>,
}

impl Context {
    /// Queues a function to be executed on the GUI thread when next possible. Returns
    /// immediately, not waiting for the function to be executed.
    ///
    /// # Example
    ///
    /// ```
    /// use iui::prelude::*;
    ///
    /// let ui = UI::init().unwrap();
    ///
    /// ui.queue_main(|| { println!("Runs first") } );
    /// ui.queue_main(|| { println!("Runs second") } );
    /// ui.quit();
    /// ```
    pub fn queue_main<F: FnMut() + Send + 'static>(self, callback: F) {
        queue_main_unsafe(callback)
    }

    /// Spawns a new asynchronous task on the GUI thread when next possible.
    /// Returns immediately, not waiting for the task to be executed.
    /// The GUI thread will resume normal operation when the task completes
    /// or when it is awaiting. This version can be used from any thread.
    /// The `Send` restriction lets us safely use this function from
    /// other threads.
    pub fn spawn<F: Future<Output = ()> + Send + 'static>(self, future: F) {
        let arc = std::sync::Arc::new(future);
        unsafe { spawn_unsafe(arc) }
    }
}

/// Queues a function to be executed on the GUI thread with no evidence that
/// it's safe to do so yet.
pub(crate) fn queue_main_unsafe<F: FnMut() + 'static>(callback: F) {
    extern "C" fn c_callback<G: FnMut()>(data: *mut c_void) {
        unsafe {
            Box::<G>::from_raw(data as *mut G)();
        }
    }

    unsafe {
        ui_sys::uiQueueMain(
            Some(c_callback::<F>),
            Box::into_raw(Box::new(callback)) as *mut c_void,
        );
    }
}

/// The provided arc must be newly created or only held by wakers.
pub(crate) unsafe fn spawn_unsafe<F: Future<Output = ()> + 'static>(arc: Arc<F>) {
    queue_main_unsafe(move || {
        let waker = waker::make_waker(arc.clone());
        let mut ctx = std::task::Context::from_waker(&waker);
        // SAFETY:
        // The arc can only be held by wakers as pointers. So the only possibility for two &muts to
        // be created is F::poll immediately calls wake or wake_by_ref. However, this doesn't
        // immediately create a &mut, it only queues a function to run on the GUI thread. And since
        // there is only one GUI thread, the lifetime of the current &mut will end before the next
        // &mut is created.
        match F::poll(std::pin::Pin::new_unchecked(&mut *(Arc::as_ptr(&arc) as *mut F)), &mut ctx) {
            _ => ()
        }
    })
}

pub(crate) fn ui_timer_unsafe<F: FnMut()->bool + 'static>(milliseconds: i32, callback: F) {
    extern "C" fn c_callback<G: FnMut()->bool>(data: *mut c_void) -> i32 {
        unsafe {
            if !(*(data as *mut G))() {
                let _ = Box::<G>::from_raw(data as *mut G);
                return 0
            }
            1
        }
    }

    unsafe {
        ui_sys::uiTimer(
            milliseconds,
            Some(c_callback::<F>),
            Box::into_raw(Box::new(callback)) as *mut c_void,
        );
    }
}

mod waker {
    use std::mem::ManuallyDrop;
    use std::sync::Arc;
    use std::task::{RawWaker, RawWakerVTable};
    use std::future::Future;

    pub(super) unsafe fn make_waker<F: Future<Output = ()> + 'static>(arc: Arc<F>) -> std::task::Waker {
        std::task::Waker::from_raw(
            RawWaker::new(Arc::into_raw(arc) as *const (), waker_vtable::<F>())
            )
    }

    fn waker_vtable<W: Future<Output = ()> + 'static>() -> &'static RawWakerVTable {
        &RawWakerVTable::new(
            clone_raw::<W>,
            wake_raw::<W>,
            wake_by_ref_raw::<W>,
            drop_raw::<W>,
            )
    }

    unsafe fn clone_raw<T: Future<Output = ()> + 'static>(data: *const ()) -> RawWaker {
        inc_ref_count::<T>(data);
        RawWaker::new(data, waker_vtable::<T>())
    }

    unsafe fn wake_raw<T: Future<Output = ()> + 'static>(data: *const ()) {
        let arc: Arc<T> = Arc::<T>::from_raw(data as *const T);
        super::spawn_unsafe(arc)
    }

    unsafe fn wake_by_ref_raw<T: Future<Output = ()> + 'static>(data: *const ()) {
        inc_ref_count::<T>(data);
        wake_raw::<T>(data)
    }

    unsafe fn inc_ref_count<T: Future<Output = ()>>(data: *const ()) {
        let arc = ManuallyDrop::new(Arc::<T>::from_raw(data as *const T));
        let _arc_clone: ManuallyDrop<_> = arc.clone();
    }

    unsafe fn drop_raw<T: Future<Output = ()>>(data: *const ()) {
        drop(Arc::<T>::from_raw(data as *const T));
    }
}
