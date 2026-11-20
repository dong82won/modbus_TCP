"""Modbus TCP 테스트 클라이언트 실행 모듈"""

import asyncio
import time

from pymodbus.client import AsyncModbusTcpClient
from pymodbus.exceptions import ModbusException

from modbus_tcp.shared.codec import decode_detection
from modbus_tcp.shared.modbus_protocol import DATA_REG_COUNT, DATA_START_ADDR, MODBUS_CONFIG, AckStatus, RegAddr


async def wait_for_server(client: AsyncModbusTcpClient, retry_interval: float = 1.0, max_wait_sec: float | None = None) -> None:
    """서버가 TCP 포트를 열 때까지 주기적으로 접속을 재시도합니다."""
    start_time = time.monotonic()
    attempt = 0

    while True:
        attempt += 1
        if client.connected:
            return
        client.close()

        await client.connect()
        if client.connected:
            print(f"[CLIENT] 서버 연결 완료 ({MODBUS_CONFIG.target_ip}:{MODBUS_CONFIG.port}, 시도 {attempt}회)")
            return

        if max_wait_sec is not None and (time.monotonic() - start_time) >= max_wait_sec:
            raise TimeoutError(f"서버 연결 대기 시간 초과 ({max_wait_sec}s): {MODBUS_CONFIG.target_ip}:{MODBUS_CONFIG.port}")

        if attempt == 1 or attempt % 5 == 0:
            print(f"[CLIENT] 서버 대기 중 ({MODBUS_CONFIG.target_ip}:{MODBUS_CONFIG.port})... {retry_interval}초 후 재시도 (시도 #{attempt})")
        await asyncio.sleep(retry_interval)


async def cleanup_and_wait_idle(client: AsyncModbusTcpClient, device_id: int = MODBUS_CONFIG.device_id, timeout_sec: float = 5.0) -> None:
    """요청 종료 후 REQ_CMD를 0으로 초기화하고 서버가 IDLE 상태로 돌아올 때까지 대기합니다."""
    write_res = await client.write_register(address=RegAddr.REQ_CMD, value=0, device_id=device_id)
    if write_res.isError():
        raise ModbusException(f"REQ_CMD=0 쓰기 오류: {write_res}")

    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        ack_res = await client.read_holding_registers(address=RegAddr.READY_ACK, count=1, device_id=device_id)
        if not ack_res.isError() and ack_res.registers[0] == AckStatus.IDLE:
            print(f"[CLIENT][ID={device_id}] req_cmd = 0 초기화 및 서버 IDLE 상태 복귀 확인 완료\n")
            return
        await asyncio.sleep(MODBUS_CONFIG.poll_interval)

    raise TimeoutError(f"[ID={device_id}] 서버 IDLE 상태 복귀 대기 시간 초과 ({timeout_sec}s)")


async def request_tag_from_server(client: AsyncModbusTcpClient, device_id: int = MODBUS_CONFIG.device_id, timeout_sec: float = 2.0) -> None:
    write_res = await client.write_register(address=RegAddr.REQ_CMD, value=1, device_id=device_id)
    if write_res.isError():
        raise ModbusException(f"REQ_CMD=1 쓰기 오류: {write_res}")
    print(f"[CLIENT][ID={device_id}] req_cmd = 1 전송 (AprilTag 검출 요청)")

    start_time = time.monotonic()
    while True:
        ack_res = await client.read_holding_registers(address=RegAddr.READY_ACK, count=1, device_id=device_id)

        if not ack_res.isError():
            ack_val = ack_res.registers[0]
            if ack_val == AckStatus.READY:
                break
            if ack_val == AckStatus.ERROR:
                await cleanup_and_wait_idle(client, device_id=device_id)
                raise RuntimeError("서버 측 AprilTag 검출 실패 또는 범위 초과 (Ack=2)")

        if (time.monotonic() - start_time) > timeout_sec:
            await cleanup_and_wait_idle(client, device_id=device_id)
            raise TimeoutError(f"서버 응답 대기 시간 초과 ({timeout_sec}s)")

        await asyncio.sleep(MODBUS_CONFIG.poll_interval)

    res = await client.read_holding_registers(address=DATA_START_ADDR, count=DATA_REG_COUNT, device_id=device_id)
    if res.isError():
        await cleanup_and_wait_idle(client, device_id=device_id)
        raise RuntimeError(f"레지스터 읽기 실패: {res}")

    detection = decode_detection(res.registers)

    print(
        f"[CLIENT][ID={device_id}] 수신 성공 -> Tag ID: {detection.tag_id} | TCP 6-DoF(x,y,z,Rx,Ry,Rz): {tuple(round(v, 4) for v in detection.pose)}"
    )

    await cleanup_and_wait_idle(client, device_id=device_id)


async def main(device_id: int = MODBUS_CONFIG.device_id) -> None:
    client = AsyncModbusTcpClient(host=MODBUS_CONFIG.target_ip, port=MODBUS_CONFIG.port)

    try:
        await wait_for_server(client, retry_interval=1.0, max_wait_sec=None)
        await cleanup_and_wait_idle(client, device_id=device_id)

        for i in range(20):
            print(f"--- 테스트 요청 #{i + 1} (Device ID={device_id}) ---")
            try:
                if not client.connected:
                    print("[CLIENT] 연결 끊김 감지 -> 서버 재연결 시도...")
                    await wait_for_server(client, retry_interval=1.0)
                    await cleanup_and_wait_idle(client, device_id=device_id)

                await request_tag_from_server(client, device_id=device_id)

            except (ModbusException, OSError, ConnectionError) as conn_err:
                print(f"[CLIENT 통신 단절] {conn_err} -> 다음 주기에 재연결합니다.\n")
                client.close()
            except (TimeoutError, RuntimeError) as err:
                print(f"[CLIENT 요청 오류] {err}\n")

            await asyncio.sleep(1.0)
    finally:
        client.close()


def run(device_id: int = MODBUS_CONFIG.device_id) -> None:
    try:
        asyncio.run(main(device_id=device_id))
    except KeyboardInterrupt:
        print("\n[CLIENT] KeyboardInterrupt(Ctrl+C) 감지됨.")
    finally:
        print("[CLIENT] 프로그램이 정상 종료되었습니다.")


if __name__ == "__main__":
    run()
