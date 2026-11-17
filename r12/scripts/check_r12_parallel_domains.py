"""Bounded ROS isolation smoke test with two independent processes per domain.

All domains use the same absolute topic and node names. Receiving the peer is
mandatory so a broken DDS transport cannot be mistaken for successful isolation.
This tests transport isolation, not four complete simulator environments.
"""
import argparse
from collections import Counter
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
import uuid


def worker(args):
    import rclpy
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node('r12_isolation_' + args.role, enable_rosout=False)
    publisher = node.create_publisher(String, '/r12_parallel_probe/identity', 10)
    received = Counter()
    subscription = node.create_subscription(
        String, '/r12_parallel_probe/identity', lambda msg: received.update([msg.data]), 10)
    own = f'{args.token}:{args.domain}:{args.role}'
    peer = f'{args.token}:{args.domain}:{"b" if args.role == "a" else "a"}'
    deadline = time.monotonic() + 15
    graphs = set()
    try:
        while time.monotonic() < deadline:
            publisher.publish(String(data=own))
            rclpy.spin_once(node, timeout_sec=.1)
            graphs.update(name for name, _ in node.get_node_names_and_namespaces())
            time.sleep(.02)
        foreign = {key: value for key, value in received.items() if key not in (own, peer)}
        unexpected_nodes = sorted(graphs - {'r12_isolation_a', 'r12_isolation_b'})
        result = dict(domain=args.domain, role=args.role, received=dict(received),
                      peer_received=received[peer], foreign_messages=foreign,
                      unexpected_nodes=unexpected_nodes,
                      passed=received[peer] >= 10 and not foreign and not unexpected_nodes)
        Path(args.output).write_text(json.dumps(result, indent=2))
        return 0 if result['passed'] else 1
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


def main(args):
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=False)
    token = uuid.uuid4().hex
    processes, logs, sockets = [], [], []
    domains = [81, 82, 83, 84]
    ports = [20081, 20082, 20083, 20084]
    try:
        for port in ports:
            sock = socket.socket()
            sockets.append(sock)
            sock.bind(('127.0.0.1', port))
        for domain in domains:
            for role in ('a', 'b'):
                env = os.environ.copy()
                env.update(ROS_DOMAIN_ID=str(domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                           FASTDDS_BUILTIN_TRANSPORTS='UDPv4')
                logfile = (out / f'{domain}_{role}.log').open('x')
                logs.append(logfile)
                processes.append(subprocess.Popen(
                    [sys.executable, __file__, '--worker', '--domain', str(domain), '--role', role,
                     '--token', token, '--output', str(out / f'{domain}_{role}.json')],
                    env=env, stdout=logfile, stderr=subprocess.STDOUT))
        deadline = time.monotonic() + 40
        for process in processes:
            process.wait(timeout=max(.1, deadline - time.monotonic()))
        rows = [json.loads((out / f'{domain}_{role}.json').read_text())
                for domain in domains for role in ('a', 'b')]
        result = dict(domains=domains, tcp_ports=ports, ports_bindable_concurrently=True,
                      probe_processes=8, complete_simulators_launched=0, rows=rows,
                      passed=all(p.returncode == 0 for p in processes) and all(r['passed'] for r in rows),
                      scope='Same-name ROS topic isolation with peer delivery, not simulation performance')
        (out / 'summary.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result))
        return 0 if result['passed'] else 1
    finally:
        for process in processes:
            if process.poll() is None:
                process.terminate()
        for process in processes:
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)
        for logfile in logs:
            logfile.close()
        for sock in sockets:
            sock.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--worker', action='store_true')
    parser.add_argument('--domain', type=int)
    parser.add_argument('--role', choices=('a', 'b'))
    parser.add_argument('--token')
    args = parser.parse_args()
    raise SystemExit(worker(args) if args.worker else main(args))
