"""Bounded same-thread scheduling only; no receipt or settling policy changes."""
import select,time

def wait_publication(ros,decoder,lower,*,clock=time.monotonic_ns,wait=select.select):
    while True:
        remaining=500_000_000-(clock()-lower)
        if remaining<0 or remaining>500_000_000:raise ValueError('publication_RPC_capture_expired')
        sockets=[ros]+([] if decoder.refreshed_once or getattr(decoder,'pending_terminal',None)is not None else [decoder.conn])
        ready,_,_=wait(sockets,[],[],remaining/1e9)
        if not ready:raise ValueError('publication_RPC_capture_expired')
        # Service ready RPC first, even when the ROS completion is simultaneously ready.
        if decoder.conn in ready and not decoder.refreshed_once:
            previous=decoder.conn.gettimeout()
            decoder.conn.settimeout(max(1e-9,(500_000_000-(clock()-lower))/1e9))
            try:decoder.refresh_during_publication()
            finally:decoder.conn.settimeout(previous)
        if ros in ready:return
