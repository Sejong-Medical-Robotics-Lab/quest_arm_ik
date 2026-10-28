# fakeros — ROS 없이 노드를 한 프로세스에서 돌리는 가짜 rclpy

`test/test_hr_chain.py` 가 쓴다. 토픽 라우팅·타이머·서비스·파라미터 오버라이드·가짜 단조시계만 구현한 최소판이며,
`ros2_interfaces` 는 SH-Humanoid 저장소의 `integration/ros2_interfaces/*.msg|*.srv` 를 읽어 클래스로 만든다 (`SH_REPO` 필요).
실제 rclpy 가 설치된 환경에서도 `sys.path` 앞에 끼워 넣어 쓰므로 테스트는 항상 가짜 쪽을 쓴다.
