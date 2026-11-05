"""Request X11 WM_DELETE_WINDOW only for windows belonging to the owned PID."""
import ctypes as C
import ctypes.util
import time

def close_owned_window(pid, *, save_config=False):
 x=C.CDLL(ctypes.util.find_library('X11'));U=C.c_ulong;P=C.c_void_p
 x.XOpenDisplay.argtypes=[C.c_char_p];x.XOpenDisplay.restype=P
 x.XDefaultRootWindow.argtypes=[P];x.XDefaultRootWindow.restype=U
 x.XInternAtom.argtypes=[P,C.c_char_p,C.c_int];x.XInternAtom.restype=U
 x.XQueryTree.argtypes=[P,U,C.POINTER(U),C.POINTER(U),C.POINTER(C.POINTER(U)),C.POINTER(C.c_uint)]
 x.XGetWindowProperty.argtypes=[P,U,U,C.c_long,C.c_long,C.c_int,U,C.POINTER(U),C.POINTER(C.c_int),C.POINTER(U),C.POINTER(U),C.POINTER(C.POINTER(C.c_ubyte))]
 x.XFree.argtypes=[P];x.XCloseDisplay.argtypes=[P];x.XFlush.argtypes=[P]
 class Data(C.Union):_fields_=[('l',C.c_long*5),('b',C.c_char*20)]
 class Message(C.Structure):_fields_=[('type',C.c_int),('serial',U),('send_event',C.c_int),('display',P),('window',U),('message_type',U),('format',C.c_int),('data',Data)]
 class Key(C.Structure):_fields_=[('type',C.c_int),('serial',U),('send_event',C.c_int),('display',P),('window',U),('root',U),('subwindow',U),('time',U),('x',C.c_int),('y',C.c_int),('x_root',C.c_int),('y_root',C.c_int),('state',C.c_uint),('keycode',C.c_uint),('same_screen',C.c_int)]
 class Event(C.Union):_fields_=[('message',Message),('key',Key),('pad',C.c_long*24)]
 x.XFetchName.argtypes=[P,U,C.POINTER(C.c_char_p)]
 x.XKeysymToKeycode.argtypes=[P,U];x.XKeysymToKeycode.restype=C.c_uint
 x.XSendEvent.argtypes=[P,U,C.c_int,C.c_long,C.POINTER(Event)]
 d=x.XOpenDisplay(None)
 if not d:return {'requested':False,'reason':'no_x11_display'}
 matched=[]
 try:
  atom=x.XInternAtom(d,b'_NET_WM_PID',False);protocol=x.XInternAtom(d,b'WM_PROTOCOLS',False);delete=x.XInternAtom(d,b'WM_DELETE_WINDOW',False)
  queue=[x.XDefaultRootWindow(d)];seen=set()
  while queue and len(seen)<10000:
   w=queue.pop()
   if w in seen:continue
   seen.add(w);actual=U();fmt=C.c_int();n=U();remaining=U();data=C.POINTER(C.c_ubyte)()
   status=x.XGetWindowProperty(d,w,atom,0,1,False,6,C.byref(actual),C.byref(fmt),C.byref(n),C.byref(remaining),C.byref(data))
   try:
    if status==0 and actual.value==6 and fmt.value==32 and n.value==1 and C.cast(data,C.POINTER(U))[0]==pid:matched.append(w)
   finally:
    if data:x.XFree(data)
   root=U();parent=U();children=C.POINTER(U)();count=C.c_uint()
   if x.XQueryTree(d,w,C.byref(root),C.byref(parent),C.byref(children),C.byref(count)):
    queue.extend(children[i] for i in range(count.value))
    if children:x.XFree(children)
  main=[];titles={}
  for w in matched:
   title=C.c_char_p()
   if x.XFetchName(d,w,C.byref(title)) and title.value:
    text=title.value.decode('utf-8',errors='replace');titles[str(w)]=text;x.XFree(C.cast(title,P))
    if text.endswith(' - RViz') or text in ('RViz','RViz*'):main.append(w)
  if len(main)!=1:return {'requested':False,'reason':'main_window_not_unique','titles':titles,'owned_pid':pid}
  for w in main:
   if save_config:
    for kind in (2,3):
     e=Event();e.key.type=kind;e.key.display=d;e.key.window=w;e.key.root=x.XDefaultRootWindow(d);e.key.state=4;e.key.keycode=x.XKeysymToKeycode(d,ord('s'));e.key.same_screen=1
     x.XSendEvent(d,w,False,1 if kind==2 else 2,C.byref(e))
    x.XFlush(d);time.sleep(.5)
   e=Event();e.message.type=33;e.message.display=d;e.message.window=w;e.message.message_type=protocol;e.message.format=32;e.message.data.l[0]=delete;e.message.data.l[1]=0
   x.XSendEvent(d,w,False,0,C.byref(e))
  x.XFlush(d);return {'requested':bool(matched),'owned_pid':pid,'windows':main,'titles':titles,'save_requested':save_config}
 finally:x.XCloseDisplay(d)
