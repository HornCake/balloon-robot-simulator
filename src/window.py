"""Initial native viewer sizing; offscreen framebuffer sizes are unrelated."""
from contextlib import contextmanager
from concurrent.futures import Future
import ctypes
import platform
import sys


_pending = []


def _maximize_cocoa_window():
    objc = ctypes.CDLL('/usr/lib/libobjc.A.dylib')
    objc.objc_getClass.argtypes = [ctypes.c_char_p]
    objc.objc_getClass.restype = ctypes.c_void_p
    objc.sel_registerName.argtypes = [ctypes.c_char_p]
    objc.sel_registerName.restype = ctypes.c_void_p

    def send(receiver, selector, restype=ctypes.c_void_p, types=(), values=()):
        function = ctypes.CFUNCTYPE(restype, ctypes.c_void_p, ctypes.c_void_p, *types)(('objc_msgSend', objc))
        return function(receiver, objc.sel_registerName(selector.encode()), *values)

    app = send(objc.objc_getClass(b'NSApplication'), 'sharedApplication')
    window = send(app, 'keyWindow') or send(app, 'mainWindow')
    if not window:
        windows = send(app, 'orderedWindows')
        if send(windows, 'count', ctypes.c_ulong):
            window = send(windows, 'objectAtIndex:', types=(ctypes.c_ulong,), values=(0,))
    if not window:
        raise RuntimeError('The viewer window is not available')
    screen = send(window, 'screen') or send(objc.objc_getClass(b'NSScreen'), 'mainScreen')

    class Rect(ctypes.Structure):
        _fields_ = [(name, ctypes.c_double) for name in ('x', 'y', 'width', 'height')]

    if platform.machine() == 'x86_64':
        frame = Rect()
        function = ctypes.CFUNCTYPE(None, ctypes.POINTER(Rect), ctypes.c_void_p, ctypes.c_void_p)(('objc_msgSend_stret', objc))
        function(ctypes.byref(frame), screen, objc.sel_registerName(b'visibleFrame'))
    else:
        frame = send(screen, 'visibleFrame', Rect)
    send(window, 'setFrame:display:', None, (Rect, ctypes.c_bool), (frame, True))
    return frame.width, frame.height


@ctypes.CFUNCTYPE(None, ctypes.c_void_p)
def _on_main_queue(_):
    future = _pending.pop(0)
    try:
        future.set_result(_maximize_cocoa_window())
    except Exception as error:
        future.set_exception(error)
        print(f'Could not enlarge viewer window: {error}', file=sys.stderr)


def maximize_viewer_window():
    """Run Cocoa window operations on mjpython's main/UI thread."""
    if sys.platform != 'darwin':
        return None  # GLFW's creation hint handles other desktop platforms.
    dispatch = ctypes.CDLL('/usr/lib/system/libdispatch.dylib')
    queue = ctypes.addressof(ctypes.c_byte.in_dll(dispatch, '_dispatch_main_q'))
    dispatch.dispatch_async_f.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p]
    future = Future()
    _pending.append(future)
    dispatch.dispatch_async_f(queue, None, ctypes.cast(_on_main_queue, ctypes.c_void_p))
    return future


@contextmanager
def startup_window(maximized=True):
    glfw = None
    if maximized and sys.platform != 'darwin':
        import glfw
        if not glfw.init():
            raise RuntimeError('Could not initialize GLFW')
        glfw.window_hint(glfw.MAXIMIZED, glfw.TRUE)
    try:
        yield maximize_viewer_window if maximized else lambda: None
    finally:
        if glfw is not None:
            glfw.window_hint(glfw.MAXIMIZED, glfw.FALSE)
