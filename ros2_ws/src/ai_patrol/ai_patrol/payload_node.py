#!/usr/bin/env python3
"""救命浮環の投下装置(シミュレーション)。

/ai_patrol/drop_request(JSON: name, north, east, alt, v_north, v_east)を受けると、
機体の真下に救命浮環を出現させ、自由落下させて海面に浮かべ、以降は海流で漂流させる。

Gazebo の水面には浮力の計算が無いため、浮環は「重力なし・当たり判定なし」のモデルにし、
落下と漂流の動きをこのノードが set_pose で与える(z = h - g t^2 / 2 → 海面で停止)。
gz-transport の Python バインディングを使う(システムの python3 で動かすこと)。
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode

G = 9.8
FLOAT_Z = 0.08          # 海面に浮いたときの高さ [m]


def ring_sdf(name, radius=0.36, n=16):
    """救命浮環(オレンジと白の輪)。SDF にトーラスが無いので小さな円柱を輪状に並べる。"""
    parts = []
    for k in range(n):
        a = 2 * math.pi * k / n
        col = '1 0.35 0.05 1' if (k // 2) % 2 == 0 else '0.95 0.95 0.95 1'
        parts.append(f"""<visual name="s{k}"><pose>{radius * math.cos(a):.3f} {radius * math.sin(a):.3f} 0 1.5708 0 {a + math.pi / 2:.3f}</pose>
          <geometry><cylinder><radius>0.07</radius><length>{2 * math.pi * radius / n * 1.15:.3f}</length></cylinder></geometry>
          <material><diffuse>{col}</diffuse><ambient>{col}</ambient><emissive>0.2 0.08 0 1</emissive></material></visual>""")
    return f"""<sdf version="1.9"><model name="{name}"><link name="link"><gravity>false</gravity>
      <inertial><mass>2.5</mass><inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.2</izz></inertia></inertial>
      {''.join(parts)}</link></model></sdf>"""


class PayloadNode(Node):
    def __init__(self):
        super().__init__('ai_payload')
        self.declare_parameter('world', 'sea_sar')
        self.world = self.get_parameter('world').value
        self.gz = GzNode()
        self.rings = {}           # name → dict(t0, n0, e0, h0, vn, ve, spawned)
        self.create_subscription(String, '/ai_patrol/drop_request', self._on_drop, 10)
        self.create_timer(0.05, self._animate)
        self.get_logger().info('救命浮環の投下装置: 待機中')

    def _on_drop(self, msg):
        d = json.loads(msg.data)
        name = d['name']
        h0 = max(0.5, d['alt'] - 0.4)        # 機体の下面から落とす
        req = EntityFactory()
        req.sdf = ring_sdf(name)
        req.name = name
        req.pose.position.x, req.pose.position.y, req.pose.position.z = d['east'], d['north'], h0
        req.pose.orientation.w = 1.0
        ok, rep = self.gz.request(f'/world/{self.world}/create', req, EntityFactory, Boolean, 3000)
        if not (ok and rep.data):
            self.get_logger().error(f'救命浮環の生成に失敗: {name}')
            return
        self.rings[name] = dict(t0=time.time(), n0=d['north'], e0=d['east'], h0=h0,
                                vn=d['v_north'], ve=d['v_east'])
        self.get_logger().info(f'{d["id"]}: 救命浮環 {name} を投下 (高度 {h0:.1f} m)')

    def _animate(self):
        if not self.rings:
            return
        now = time.time()
        req = Pose_V()
        for name, r in self.rings.items():
            t = now - r['t0']
            z = max(FLOAT_Z, r['h0'] - 0.5 * G * t * t)
            p = req.pose.add()
            p.name = name
            p.position.x = r['e0'] + r['ve'] * t       # 落下中も着水後も海流で流れる
            p.position.y = r['n0'] + r['vn'] * t
            p.position.z = z
            p.orientation.w = 1.0
        self.gz.request(f'/world/{self.world}/set_pose_vector', req, Pose_V, Boolean, 500)


def main(args=None):
    rclpy.init(args=args)
    node = PayloadNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
