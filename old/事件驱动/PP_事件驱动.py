import logging
import sys
import time
from typing import Dict, Optional

try:
	from .bus import CommunicationBus
	from .motor_pp import PPMotor
	from .controller import SupervisoryController
except ImportError:
	import os as _os
	_HERE = _os.path.dirname(__file__)
	if _HERE not in sys.path:
		sys.path.insert(0, _HERE)
	from bus import CommunicationBus  
	from motor_pp import PPMotor 
	from controller import SupervisoryController 


logging.basicConfig(
	level=logging.INFO,
	format='[%(asctime)s] %(levelname)s %(name)s: %(message)s',
	datefmt='%H:%M:%S'
)
logger = logging.getLogger("PP_事件驱动")


import os

EDS_FILE = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'ZeroErr Driver_V1.5.eds')


def parse_pairs(line: str) -> Dict[int, float]:
	"""解析用户输入，例如：'01-10 02-25' -> {1: 10.0, 2: 25.0}
	节点 ID 支持带前导零的十进制（也可写 0x.. 十六进制）。
	"""
	res: Dict[int, float] = {} # 存储解析结果，res 变量应该是一个键为整数、值为浮点数的字典
	for token in line.strip().split():
		if '-' not in token:
			continue
		nid_str, ang_str = token.split('-', 1)
		try:
			node_id = int(nid_str, 16) if nid_str.startswith('0x') else int(nid_str, 10)
			angle = float(ang_str)
			res[node_id] = angle
		except ValueError:
			logger.warning("无法解析指令片段: %s", token)
	return res


def main():
	# 命令行参数：可选 SYNC 生产者 ID、节点列表等
	# 示例：python PP_事件驱动.py --sync 1 --nodes 1,2,3 --period 10
	import argparse

	parser = argparse.ArgumentParser(description="PP 事件驱动多轴同步控制")
	parser.add_argument('--interface', default='pcan')
	parser.add_argument('--channel', default='PCAN_USBBUS1')
	parser.add_argument('--bitrate', type=int, default=1_000_000)
	parser.add_argument('--nodes', type=str, default='1,4', help='逗号分隔的节点ID，例如 1,2,3')
	parser.add_argument('--sync', type=int, default=4, help='作为 SYNC 生产者的节点ID')
	parser.add_argument('--period', type=int, default=20, help='SYNC 周期(ms)')
	args = parser.parse_args()

	node_ids = []
	for s in args.nodes.split(','):
		s = s.strip()
		if not s:
			continue
		node_ids.append(int(s, 10))

	bus = CommunicationBus(args.interface, args.channel, args.bitrate)
	bus.connect()

	# 创建电机对象
	motors: Dict[int, PPMotor] = {}
	for nid in node_ids:
		motors[nid] = PPMotor(bus, nid, EDS_FILE, args.period)

	# 确保 SYNC 生产者节点也在电机集合中，便于统一控制
	if args.sync not in motors:
		motors[args.sync] = PPMotor(bus, args.sync, EDS_FILE, args.period)

	controller = SupervisoryController(bus, motors, sync_producer_id=args.sync, sync_period_ms=args.period)
	controller.start()

	print("\n输入指令示例: 01-10 02-25 (表示 1 号电机到10°, 2号电机到25°)")
	print("输入: sync <id> [period_ms] 可切换SYNC生产者，例如: sync 2 20")
	print("支持连续输入，按 Ctrl+C 退出。")

	try:
		while True:
			line = input("> ").strip()
			if not line:
				continue
			if line.lower() in ("q", "quit", "exit"):
				break

			# 特殊指令：sync <id> [period]
			if line.lower().startswith('sync'):
				parts = line.split()
				if len(parts) >= 2:
					try:
						new_sync_id = int(parts[1], 10)
						if new_sync_id not in motors:
							motors[new_sync_id] = PPMotor(bus, new_sync_id, EDS_FILE, args.period)
							motors[new_sync_id].basic_init_pp()
							motors[new_sync_id].enable()
						new_period = args.period
						if len(parts) >= 3:
							new_period = int(parts[2], 10)
						bus.configure_sync_producer(new_sync_id, new_period)
						print(f"已切换 SYNC 生产者为节点 {new_sync_id}，周期 {new_period}ms")
						continue
					except Exception as e:
						print(f"切换 SYNC 失败: {e}")
						continue

			angles = parse_pairs(line)
			if not angles:
				print("未解析到有效指令。格式: 01-10 02-25")
				continue
			# 确保所有被引用的电机已创建并使能
			for nid in list(angles.keys()):
				if nid not in motors:
					try:
						motors[nid] = PPMotor(bus, nid, EDS_FILE, args.period)
						motors[nid].basic_init_pp()
						motors[nid].enable()
						print(f"已添加并初始化节点 {nid}")
					except Exception as e:
						print(f"添加节点 {nid} 失败: {e}")
						angles.pop(nid, None)
			controller.set_angles_next_sync(angles)
	except KeyboardInterrupt:
		pass
	finally:
		try:
			controller.stop()
		finally:
			bus.disconnect()


if __name__ == "__main__":
	main()
