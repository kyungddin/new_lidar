# 250707
1. intensity 기반으로 point cloud를 paint하거나, 노이즈를 제거하는 등 제어를 해봄
2. intensity(refletivity) 기반으로 거울이나, 거울 너머의 세계를 탐지해봤지만, 큰 특이점 없었음
3. 다만 거울 너머의 세계는 그 반사되는 물체들의 포인트 클라우드를 둘러싸는 테두리가 포인트 클라우드가 아예 없는 특이사항이 존재 -> 이를 기반으로 클러스터링을 하고 라벨링이 가능할 것으로 예상이 됨
4. 일단 관련 논문들을 훑어보며, 중복되는 아이디어를 제거하자

# 250713

1. dual return에 대해서 계속해서 실시간 시각화를 해보려고 노력했으나 결국 실패
2. 그 이유를 분석한 결과 dual return packet 처리가 python과 같은 High Level에선 쉽지 않았음
3. 따라서 결과적으로 parsing을 계속 시도해도 strong return만 들어오는 상황
4. 굳이 dual return을 쓰고 싶다면 실시간 처리보다는, ouster studio를 통해 녹화한 후 그것을 후처리하는 방향으로 가야할 듯

# 250714

1. 일단 시각화는 확실히 rviz로 하는 것이 맞아 보이고..
2. dual return은 거울을 다루는 것에 있어서 확실히 필요해 보이긴 하다
3. 다만 이 경우 point cloud를 받아오는 작업을 하는 것에 있어 PCL을 사용하는 것이 불가피해보임
4. 좋은 발견: 거울을 둘러싼 테두리는 strong return, 그리고 거울은 last return으로 이 둘을 기반으로 확실히 거울은 detection 할 수 있다

5. ouster_ros에서 무려 자동으로 dual return을 분리해준다. 이 topic을 구독해서 open_3d에서 처리하면 되는데
6. 역시 ros는 다루기가 여간 까다롭다. 일단은 그래도 발견을 했으니 굿

# 250724

1. Ouster의 Lidar는 기본적으로 dual return에 약한 라이다임을 확인했다.. dual return을 계속 진행하는 것이 맞는지에 대한 고민..
2. 그렇다면 다른 물리량만을 이용해서 거울을 탐지하고 이용해볼까? ouster lidar만의 장점을 찾기?
3. 아니면 기존의 dataset을 이용하는 연구도 괜찮을 거 같기도 하다
