# Autonomous_flight

시도해보려는 알고리즘 : A*, DBF 알고리즘
PCL로부터 cartesian 좌표계로 구성된 지도를 만들고, 지도에서 장애물 segmentation 진행.
그리고 알고리즘을 활용해 최단경로 알고리즘을 path에 저장하고 list형태로 PX4_Autopilot에 publish

<현재 진행상황>
[1월 12일]
2, 3차원 랜덤으로 생성되는 맵에 대해서 기본적인 A* 알고리즘을 구현해봄.
PCL로부터 x,y,z 좌표계로 변환하는 것도 구현 완료.
이후 segmentation을 하고, 어디를 시작 지점으로 잡고 도착지점을 yolo를 통해 어떻게 결정할지 이거도 해봐야겠다.
그리고 동적으로 시간이 지남에 따라서 새로 맵 작성해서 경로 수정해가는 것도 만들어야 한다. 
Structure code 구현완료
