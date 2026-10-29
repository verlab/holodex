#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

def main():
    rclpy.init()
    node = Node('map_saver_cli')
    # Suporta tanto /icp/save_submap quanto o alias legado /icp/save_map
    client = node.create_client(Trigger, '/icp/save_submap')
    print("Aguardando servico /icp/save_submap...")
    if not client.wait_for_service(timeout_sec=3.0):
        client = node.create_client(Trigger, '/icp/save_map')
        print("Tentando servico alternativo /icp/save_map...")
        if not client.wait_for_service(timeout_sec=2.0):
            print("Erro: Servico /icp/save_submap ou /icp/save_map nao encontrado!")
            sys.exit(1)

    req = Trigger.Request()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.done():
        res = future.result()
        if res.success:
            print(f"[SUCESSO] {res.message}")
        else:
            print(f"[ERRO] {res.message}")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
