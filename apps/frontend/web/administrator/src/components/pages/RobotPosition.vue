<template>
    <div class="robot-position-page">
     
  
     <!-- 카카오맵 api 넣을부분  -->
      <div class="map-container">
        <div id="map" class="kakao-map"></div>
      </div>

      <!-- 로봇 정보 패널 -->
      <div class="robot-info-panel">
        <div class="panel-header">
          <h3 class="panel-title">로봇 위치 정보</h3>
          <div class="robot-selector">
            <v-select
              v-model="selectedRobotId"
              :items="robotOptions"
              item-title="title"
              item-value="value"
              label="로봇 선택"
              variant="outlined"
              density="compact"
              hide-details
              class="robot-select"
              @update:model-value="onRobotChange"
            ></v-select>
            <v-btn
              @click="refreshRobotData"
              :loading="loading"
              variant="outlined"
              size="small"
              class="refresh-btn"
            >
              <v-icon>mdi-refresh</v-icon>
            </v-btn>
            <v-chip 
              :color="isConnected ? 'success' : 'error'"
              size="small"
              class="connection-status"
            >
              {{ isConnected ? '연결됨' : '연결 안됨' }}
            </v-chip>
          </div>
        </div>

                 <div v-if="selectedRobot" class="robot-details">
           <div class="detail-grid">
             <div class="detail-item">
               <div class="detail-label">Current State</div>
               <div class="detail-value">{{ selectedRobot.statusDisplayName }}</div>
             </div>
             <div class="detail-item">
               <div class="detail-label">Last Charge Time</div>
               <div class="detail-value">{{ formatDate(selectedRobot.lastChargeTime) }}</div>
             </div>
             <div class="detail-item">
               <div class="detail-label">Latitude</div>
               <div class="detail-value">{{ selectedRobot.latitude || 'N/A' }}</div>
             </div>
             <div class="detail-item">
               <div class="detail-label">Longitude</div>
               <div class="detail-value">{{ selectedRobot.longitude || 'N/A' }}</div>
             </div>
           </div>
         </div>

        <div v-else class="no-robot-selected">
          <v-icon size="48" color="#8a92a6">mdi-robot</v-icon>
          <p>로봇을 선택해주세요</p>
        </div>
      </div>
    </div>
  </template>
  
  <script setup>
  import { ref, onMounted, computed, watch, onUnmounted } from 'vue'
  
  // Reactive data
const loading = ref(false)
const selectedRobotId = ref(null)
const allRobots = ref([])
const selectedRobot = ref(null)
const map = ref(null)
const robotMarker = ref(null)
const isConnected = ref(false)

// SSE 관련
let eventSource = null

  // Computed
  const robotOptions = computed(() => {
    console.log('robotOptions 계산 중, allRobots:', allRobots.value)
    const options = allRobots.value.map(robot => ({
      title: `${robot.code} `,
      value: robot.id
    }))
    console.log('생성된 옵션:', options)
    return options
  })

  // Methods

// SSE 연결 시작
const startSSEConnection = () => {
  try {
    console.log('SSE 연결 시작...')
    loading.value = true
    
    // 기존 연결이 있으면 닫기
    if (eventSource) {
      eventSource.close()
    }
    
    // SSE 연결 생성
    eventSource = new EventSource('/api/v1/robots/location/subscribe')
    
    // 연결 성공
    eventSource.onopen = (event) => {
      console.log('SSE 연결 성공')
      isConnected.value = true
      loading.value = false
    }
    
    // robotLocations 이벤트 수신
    eventSource.addEventListener('robotLocations', (event) => {
      try {
        const robots = JSON.parse(event.data)
        console.log('로봇 위치 데이터 수신:', robots)
        
        // 로봇 목록 업데이트
        allRobots.value = robots
        
        // 첫 번째 로봇을 기본 선택 (처음 로드 시에만)
        if (robots.length > 0 && !selectedRobotId.value) {
          selectedRobotId.value = robots[0].id
          selectedRobot.value = robots[0]
          console.log('첫 번째 로봇 선택:', selectedRobotId.value)
        }
        
        // 현재 선택된 로봇 정보 업데이트
        if (selectedRobotId.value) {
          const currentRobot = robots.find(r => r.id === selectedRobotId.value)
          if (currentRobot) {
            selectedRobot.value = currentRobot
          }
        }
        
      } catch (error) {
        console.error('로봇 위치 데이터 파싱 실패:', error)
      }
    })
    
    // 일반 메시지 수신
    eventSource.onmessage = (event) => {
      console.log('일반 메시지 수신:', event.data)
    }
    
    // 에러 처리
    eventSource.onerror = (event) => {
      console.error('SSE 연결 오류:', event)
      isConnected.value = false
      loading.value = false
    }
    
  } catch (error) {
    console.error('SSE 연결 시작 실패:', error)
    loading.value = false
  }
}

// SSE 연결 중지
const stopSSEConnection = () => {
  if (eventSource) {
    eventSource.close()
    eventSource = null
  }
  isConnected.value = false
  console.log('SSE 연결 중지')
}

const onRobotChange = (robotId) => {
  if (robotId) {
    // 전체 목록에서 해당 로봇 찾기
    const robot = allRobots.value.find(r => r.id === robotId)
    if (robot) {
      selectedRobot.value = robot
    }
  } else {
    selectedRobot.value = null
  }
}

const refreshRobotData = () => {
  // SSE 연결 재시작
  stopSSEConnection()
  startSSEConnection()
}

  const getRobotPosition = () => {
    if (!selectedRobot.value || !selectedRobot.value.locationAvailable) {
      return {}
    }
    
    // 위도/경도를 지도 좌표로 변환 (간단한 예시)
    const lat = selectedRobot.value.latitude
    const lng = selectedRobot.value.longitude
    
    // 실제로는 지도 API의 좌표 변환 함수를 사용해야 함
    // 여기서는 임시로 고정 위치 사용
    return {
      top: '30%',
      left: '50%'
    }
  }

  const getStatusClass = (status) => {
    switch (status) {
      case 'WORKING':
        return 'working'
      case 'WAITING':
        return 'waiting'
      case 'CHARGING':
        return 'charging'
      case 'ERROR':
        return 'error'
      default:
        return 'unknown'
    }
  }

  const formatDate = (dateString) => {
    if (!dateString) return '정보 없음'
    return new Date(dateString).toLocaleString('ko-KR')
  }

  const formatLastUpdated = (dateString) => {
    if (!dateString) return '정보 없음'
    const date = new Date(dateString)
    const now = new Date()
    const diffMs = now - date
    const diffMins = Math.floor(diffMs / (1000 * 60))
    
    if (diffMins < 1) return '방금 전'
    if (diffMins < 60) return `${diffMins}분 전`
    if (diffMins < 1440) return `${Math.floor(diffMins / 60)}시간 전`
    return date.toLocaleDateString('ko-KR')
  }

  // 카카오맵 초기화
  const initKakaoMap = () => {
    if (window.kakao && window.kakao.maps) {
      const container = document.getElementById('map')
      const options = {
        center: new window.kakao.maps.LatLng(37.5565, 126.972), // 서울 시청 근처
        level: 3
      }
      
      map.value = new window.kakao.maps.Map(container, options)
      console.log('카카오맵 초기화 완료')
    }
  }

  // 로봇 마커 업데이트
  const updateRobotMarker = () => {
    if (!map.value || !selectedRobot.value || !selectedRobot.value.locationAvailable) {
      if (robotMarker.value) {
        robotMarker.value.setMap(null)
        robotMarker.value = null
      }
      return
    }

    const position = new window.kakao.maps.LatLng(
      selectedRobot.value.latitude,
      selectedRobot.value.longitude
    )

    // 기존 마커 제거
    if (robotMarker.value) {
      robotMarker.value.setMap(null)
    }

    // 새 마커 생성
    const markerImage = new window.kakao.maps.MarkerImage(
      'data:image/svg+xml;base64,PHN2ZyB3aWR0aD0iMjQiIGhlaWdodD0iMjQiIHZpZXdCb3g9IjAgMCAyNCAyNCIgZmlsbD0ibm9uZSIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIj4KPHBhdGggZD0iTTEyIDJDNi40OCAyIDIgNi40OCAyIDEyQzIgMTcuNTIgNi40OCAyMiAxMiAyMkMxNy41MiAyMiAyMiAxNy41MiAyMiAxMkMyMiA2LjQ4IDE3LjUyIDIgMTIgMloiIGZpbGw9IiMzYTU3ZTgiLz4KPHBhdGggZD0iTTEyIDZDNi40OCA2IDIgMTAuNDggMiAxNkMyIDIxLjUyIDYuNDggMjYgMTIgMjZDMjEuNTIgMjYgMjYgMjEuNTIgMjYgMTZDMjYgMTAuNDggMjEuNTIgNiAxMiA2WiIgZmlsbD0iIzNhNTdlOCIgZmlsbC1vcGFjaXR5PSIwLjMiLz4KPC9zdmc+',
      new window.kakao.maps.Size(24, 24)
    )

    robotMarker.value = new window.kakao.maps.Marker({
      position: position,
      map: map.value,
      image: markerImage
    })

    // SSE 연결은 이미 상위에서 관리되므로 여기서는 제거

    // 마커 클릭 시 정보창 표시
    const infowindow = new window.kakao.maps.InfoWindow({
      content: `
        <div style="padding: 30px; min-width: 200px; border-radius: 10px; background-color: #222738;">
          <h3 style="margin: 0 0 8px 0; color: #3a57e8;">${selectedRobot.value.code}</h3>
          <p style="margin: 4px 0; color: #666;">상태: ${selectedRobot.value.statusDisplayName}</p>
          <p style="margin: 4px 0; color: #666;">위치: ${selectedRobot.value.locationDisplay}</p>
          
        </div>
      `// 여기에 주문 정보랑 예상소요시간(이건로직을 고객의 위치 기준으로 현재 로봇위치기준 직선거리
      //계산하는로직을 사용해야함함)
    })

    window.kakao.maps.event.addListener(robotMarker.value, 'click', function() {
      infowindow.open(map.value, robotMarker.value)
    })

    // 지도 중심을 로봇 위치로 이동
    map.value.setCenter(position)
  }

  // 로봇 선택 변경 시 마커 업데이트
  watch(selectedRobot, () => {
    updateRobotMarker()
  }, { deep: true })

  // 카카오맵 스크립트 로드
  const loadKakaoMapScript = () => {
    const script = document.createElement('script')
    script.src = `//dapi.kakao.com/v2/maps/sdk.js?appkey=da17381f8255309ebdec1286b22aa97f&autoload=false`
    script.onload = () => {
      window.kakao.maps.load(() => {
        initKakaoMap()
      })
    }
    document.head.appendChild(script)
  }

  // Lifecycle
onMounted(() => {
  loadKakaoMapScript()
  startSSEConnection()
})

onUnmounted(() => {
  stopSSEConnection()
})
  </script>
  
  <style scoped>
  .robot-position-page {
    padding-top: 20px;
    height: 100%;
    background-color: #081028;
    color: #ffffff;
    display: flex;
    flex-direction: column;
  }
  
  /* 헤더 */
  .page-header {
    margin-bottom: 20px;
  }
  
  .page-title {
    font-size: 24px;
    font-weight: bold;
    margin: 0;
    color: #ffffff;
  }
  
  /* 지도 컨테이너 */
  .map-container {
    width: 800px;
    height: 600px;
    background-color: #1a1f2e;
    border-radius: 12px;
    overflow: hidden;
    margin-bottom: 20px;
    align-self: center;
  }

  .kakao-map {
    width: 100%;
    height: 100%;
  }
  
  .map {
    width: 100%;
    height: 100%;
    min-height: 500px;
    position: relative;
    background-color: #0f141f;
  }
  
  .map-background {
    width: 100%;
    height: 100%;
    position: relative;
    background: linear-gradient(135deg, #1a1f2e 0%, #0f141f 100%);
  }
  
  /* 도로 네트워크 */
  .road-network {
    position: absolute;
    width: 100%;
    height: 100%;
  }
  
  .road {
    position: absolute;
    background-color: #374151;
    border-radius: 2px;
  }
  
  .horizontal-road {
    top: 50%;
    left: 0;
    width: 100%;
    height: 4px;
    transform: translateY(-50%);
  }
  
  .vertical-road {
    top: 0;
    left: 50%;
    width: 4px;
    height: 100%;
    transform: translateX(-50%);
  }
  
  .diagonal-road {
    top: 20%;
    left: 20%;
    width: 60%;
    height: 4px;
    transform: rotate(45deg);
  }
  
  /* 건물들 */
  .buildings {
    position: absolute;
    width: 100%;
    height: 100%;
  }
  
  .building {
    position: absolute;
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 4px;
    cursor: pointer;
    transition: transform 0.3s ease;
  }
  
  .building:hover {
    transform: scale(1.1);
  }
  
  .building-icon {
    font-size: 20px;
    filter: drop-shadow(0 2px 4px rgba(0, 0, 0, 0.3));
  }
  
  .building-label {
    font-size: 10px;
    color: #b6bace;
    text-align: center;
    max-width: 80px;
    line-height: 1.2;
    background-color: rgba(0, 0, 0, 0.7);
    padding: 2px 4px;
    border-radius: 4px;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }
  
  /* 건물 타입별 색상 */
  .gas-station .building-icon {
    color: #f59e0b;
  }
  
  .government .building-icon {
    color: #3a57e8;
  }
  
  .supermarket .building-icon {
    color: #13c572;
  }
  
  .apartment .building-icon {
    color: #8b5cf6;
  }
  
  .school .building-icon {
    color: #ec4899;
  }
  
  .river .building-icon {
    color: #06b6d4;
  }
  
  /* 로봇 위치 */
  .robot-location {
    position: absolute;
    top: 30%;
    left: 50%;
    transform: translate(-50%, -50%);
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 4px;
    z-index: 10;
  }
  
  .robot-icon {
    font-size: 24px;
    color: #3a57e8;
    filter: drop-shadow(0 2px 4px rgba(0, 0, 0, 0.5));
  }
  
  .robot-label {
    font-size: 12px;
    color: #3a57e8;
    font-weight: bold;
    background-color: rgba(58, 87, 232, 0.2);
    padding: 2px 6px;
    border-radius: 4px;
  }
  
  /* 사람 위치 */
  .person-location {
    position: absolute;
    top: 70%;
    left: 50%;
    transform: translate(-50%, -50%);
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 4px;
    z-index: 10;
  }
  
  .person-icon {
    font-size: 20px;
    color: #13c572;
    filter: drop-shadow(0 2px 4px rgba(0, 0, 0, 0.5));
  }
  
  .person-label {
    font-size: 12px;
    color: #13c572;
    font-weight: bold;
    background-color: rgba(19, 197, 114, 0.2);
    padding: 2px 6px;
    border-radius: 4px;
  }
  
  /* 연결선 */
  .connection-line {
    position: absolute;
    top: 30%;
    left: 50%;
    width: 2px;
    height: 40%;
    background: linear-gradient(to bottom, #3a57e8, #13c572);
    transform: translateX(-50%);
    z-index: 5;
  }
  
  /* 빨간 점들 */
  .red-dot {
    position: absolute;
    width: 6px;
    height: 6px;
    background-color: #ff4757;
    border-radius: 50%;
    box-shadow: 0 0 4px rgba(255, 71, 87, 0.6);
    animation: pulse 2s infinite;
  }
  
  @keyframes pulse {
    0% {
      transform: scale(1);
      opacity: 1;
    }
    50% {
      transform: scale(1.5);
      opacity: 0.7;
    }
    100% {
      transform: scale(1);
      opacity: 1;
    }
  }

  /* 로봇 정보 패널 */
  .robot-info-panel {
    background-color: #1a1f2e;
    border-radius: 12px;
    padding: 20px;
    margin-bottom: 20px;
  }

  .panel-header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-bottom: 20px;
  }

  .panel-title {
    font-size: 18px;
    font-weight: bold;
    margin: 0;
    color: #ffffff;
  }

  .robot-selector {
    display: flex;
    gap: 12px;
    align-items: center;
  }

  .connection-status {
    margin-left: 8px;
  }

  .robot-select {
    min-width: 200px;
  }

  .refresh-btn {
    background-color: #3a57e8;
    color: white;
  }

  .robot-details {
    background-color: #222738;
    border-radius: 8px;
    padding: 16px;
  }

     .detail-grid {
     display: grid;
     grid-template-columns: 1fr 1fr;
     gap: 20px;
   }

   .detail-item {
     display: flex;
     flex-direction: column;
     gap: 8px;
     padding: 16px;
     background-color: #1a1f2e;
     border-radius: 8px;
     border-bottom: 2px solid #3a57e8;
   }

   .detail-label {
     font-size: 12px;
     color: #8a92a6;
     font-weight: 400;
     text-transform: uppercase;
     letter-spacing: 0.5px;
   }

   .detail-value {
     font-size: 16px;
     font-weight: 600;
     color: #ffffff;
   }

  .status-badge {
    padding: 4px 8px;
    border-radius: 4px;
    font-size: 12px;
    font-weight: bold;
  }

  .status-badge.working {
    background-color: rgba(19, 197, 114, 0.2);
    color: #13c572;
  }

  .status-badge.waiting {
    background-color: rgba(255, 215, 0, 0.2);
    color: #ffd700;
  }

  .status-badge.charging {
    background-color: rgba(58, 87, 232, 0.2);
    color: #3a57e8;
  }

  .status-badge.error {
    background-color: rgba(255, 71, 87, 0.2);
    color: #ff4757;
  }

  .status-badge.unknown {
    background-color: rgba(138, 146, 166, 0.2);
    color: #8a92a6;
  }

  .detail-value.online {
    color: #13c572;
  }

  .detail-value.offline {
    color: #ff4757;
  }

  .detail-value.available {
    color: #13c572;
  }

  .detail-value.unavailable {
    color: #ff4757;
  }

  .no-robot-selected {
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    padding: 40px;
    color: #8a92a6;
    text-align: center;
  }

  .no-robot-selected p {
    margin-top: 12px;
    font-size: 16px;
  }
  
  /* 컨트롤 패널 */
  .control-panel {
    display: flex;
    gap: 20px;
    background-color: #222738;
    border-radius: 12px;
    padding: 20px;
  }
  
  .panel-section {
    flex: 1;
  }
  
  .section-title {
    font-size: 16px;
    font-weight: bold;
    margin: 0 0 12px 0;
    color: #ffffff;
  }
  
  .status-item {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 8px 0;
    border-bottom: 1px solid #2a2f3e;
  }
  
  .status-label {
    font-size: 14px;
    color: #b6bace;
  }
  
  .status-value {
    font-size: 14px;
    font-weight: 500;
  }
  
  .status-value.online {
    color: #13c572;
  }
  
  .control-btn {
    display: flex;
    align-items: center;
    gap: 8px;
    width: 100%;
    padding: 8px 12px;
    margin-bottom: 8px;
    background-color: #374151;
    border: none;
    border-radius: 6px;
    color: #ffffff;
    font-size: 12px;
    cursor: pointer;
    transition: background-color 0.3s ease;
  }
  
  .control-btn:hover {
    background-color: #3a57e8;
  }

  /* Vuetify 스타일 오버라이드 */
  :deep(.v-select .v-field) {
    background-color: #2a2f3e;
    border-color: #3a3f4e;
  }

  :deep(.v-select .v-field__input) {
    color: #ffffff;
  }

  :deep(.v-select .v-field__label) {
    color: #8a92a6;
  }

  :deep(.v-btn) {
    text-transform: none;
  }
  
  /* 반응형 디자인 */
  @media (max-width: 768px) {
    .control-panel {
      flex-direction: column;
    }
    
    .map-container {
      width: 100%;
      height: 400px;
    }
    
    .building-label {
      font-size: 8px;
      max-width: 60px;
    }
    
    .robot-icon {
      font-size: 20px;
    }
    
    .person-icon {
      font-size: 18px;
    }

    .panel-header {
      flex-direction: column;
      gap: 12px;
      align-items: stretch;
    }

    .robot-selector {
      flex-direction: column;
    }

         .detail-grid {
       grid-template-columns: 1fr 1fr;
       gap: 12px;
     }
  }
  
  @media (max-width: 480px) {
    .robot-position-page {
      padding: 10px;
    }
    
    .page-title {
      font-size: 20px;
    }
    
    .control-panel {
      padding: 15px;
    }
    
    .map-container {
      width: 100%;
      height: 300px;
    }

         .robot-info-panel {
       padding: 15px;
     }

     .detail-grid {
       grid-template-columns: 1fr;
       gap: 10px;
     }

     .detail-item {
       padding: 12px;
     }
  }
  </style>