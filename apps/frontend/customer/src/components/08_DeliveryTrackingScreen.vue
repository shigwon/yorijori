<template>
  <div class="delivery-tracking-container">
    <!-- 지도 섹션 -->
    <div class="map-section">
      <div id="delivery-map" class="map-container"></div>
    </div>

    <!-- 배달 상태 카드 -->
    <div class="delivery-status-card">
      <!-- 상태 텍스트 -->
      <div class="status-text">
        <div class="status-left">
          <span class="lingki-name">LiNKY</span>
          <span class="status-message">가 가고있어요!</span>
        </div>
        <div class="time-remaining">
          <div class="time-label">남은 시간</div>
          <div class="time-value">5분</div>
        </div>
      </div>
    
      <!-- 로봇 마스코트 -->
      <div class="robot-mascot">
        <img src="../assets/homerobot.png" alt="homerobot" class="robot-image" />
      </div>
      
      <!-- 타임라인 컨테이너 -->
      <div class="timeline-container">
        <!-- 타임라인 라인 -->
        <div class="timeline-line">
          <div class="timeline-progress"></div>
        </div>
        
        <!-- 타임라인 마커 -->
        <div class="timeline-markers">
          <div class="timeline-marker completed">
            <div class="marker-dot"></div>
            <div class="marker-label">픽업완료</div>
          </div>
          
          <div class="timeline-marker current">
            <div class="marker-dot"></div>
            <div class="marker-label">배달 중</div>
          </div>
          
          <div class="timeline-marker pending">
            <div class="marker-dot"></div>
            <div class="marker-label">배달완료</div>
          </div>
        </div>
      </div>
    </div>
   
    <!-- 배달 완료 모달 -->
    <DeliveryCompleteModal 
      v-if="showDeliveryCompleteModal" 
      @close="showDeliveryCompleteModal = false"
      @show-compartment="openFoodCompartment"
    />
  </div>
</template>

<script setup>
import { onMounted, ref } from 'vue'
import { useAppState } from '../composables/useAppState'
import DeliveryCompleteModal from './09_DeliveryCompleteModal.vue'

const { openFoodCompartment, deliveryLocation, deliveryAddress, capturedImage } = useAppState()
const showDeliveryCompleteModal = ref(false)

onMounted(() => {
  console.log('DeliveryTrackingScreen 마운트됨')
  console.log('useAppState 배달 위치:', deliveryLocation.value)
  console.log('useAppState 배달 주소:', deliveryAddress.value)
  console.log('useAppState 사용자 사진:', capturedImage.value ? '있음' : '없음')
  
  // 카카오맵 초기화 함수
  const initDeliveryMap = () => {
    console.log('카카오맵 초기화 시작')
    
    if (!window.kakao || !window.kakao.maps) {
      console.error('카카오맵 API가 로드되지 않음')
      return
    }
    
    const container = document.getElementById('delivery-map')
    if (!container) {
      console.error('지도 컨테이너를 찾을 수 없음')
      return
    }
    
    // useAppState에서 저장된 위치 정보 사용
    const deliveryLat = deliveryLocation.value?.latitude || 37.5665
    const deliveryLng = deliveryLocation.value?.longitude || 126.9780
    
    console.log('지도 중심 설정:', deliveryLat, deliveryLng)
    
    try {
      const options = {
        center: new window.kakao.maps.LatLng(deliveryLat, deliveryLng),
        level: 4
      }
      
      const map = new window.kakao.maps.Map(container, options)
      
      // 목적지 마커 (사용자가 설정한 위치)
      const destPosition = new window.kakao.maps.LatLng(deliveryLat, deliveryLng)
      
      // 커스텀 마커 HTML 생성 (사용자 사진 포함)
      const userImage = capturedImage.value || ''
      const markerContent = `
        <div style="position: relative; display: inline-block;">
          <div style="
            width: 28px;
            height: 28px;
            border-radius: 50%;
            border: 2px solid white;
            box-shadow: 0 2px 8px rgba(0, 0, 0, 0.1);
            overflow: hidden;
            background: #7C3AED;
            display: flex;
            align-items: center;
            justify-content: center;
            z-index: 2;
          ">
            ${userImage ? 
              `<img src="${userImage}" alt="사용자" style="width: 100%; height: 100%; object-fit: cover;" />` : 
              '<span style="font-size: 12px; color: white;">👤</span>'
            }
          </div>
          <!-- 만날 위치 텍스트 -->
          <div style="
            position: absolute;
            top: 32px;
            left: 50%;
            transform: translateX(-50%);
            background: rgba(0, 0, 0, 0.8);
            color: white;
            padding: 3px 6px;
            border-radius: 4px;
            font-size: 10px;
            font-weight: 600;
            white-space: nowrap;
            z-index: 3;
            box-shadow: 0 2px 4px rgba(0, 0, 0, 0.2);
          ">
            만날 위치
          </div>
        </div>
      `
      
      // 커스텀 오버레이로 마커 표시
      const customOverlay = new window.kakao.maps.CustomOverlay({
        position: destPosition,
        content: markerContent,
        map: map,
        yAnchor: 0
      })
      
      // 픽업존 마커 추가 (임의 위치)
      const pickupLat = deliveryLat + 0.002 // 약간 북쪽으로
      const pickupLng = deliveryLng - 0.001 // 약간 서쪽으로
      const pickupPosition = new window.kakao.maps.LatLng(pickupLat, pickupLng)
      
      // 픽업존 마커 HTML 생성
      const pickupMarkerContent = `
        <div style="position: relative; display: inline-block;">
          <div style="
            width: 24px;
            height: 24px;
            position: relative;
            z-index: 2;
            filter: drop-shadow(0 2px 4px rgba(0, 0, 0, 0.2));
          ">
            <img src="/src/assets/pickup.png" alt="픽업존" style="width: 100%; height: 100%; object-fit: contain;" />
          </div>
          <!-- 픽업존 텍스트 -->
          <div style="
            position: absolute;
            top: 28px;
            left: 50%;
            transform: translateX(-50%);
            background: rgba(0, 0, 0, 0.8);
            color: white;
            padding: 3px 6px;
            border-radius: 4px;
            font-size: 10px;
            font-weight: 600;
            white-space: nowrap;
            z-index: 3;
            box-shadow: 0 2px 4px rgba(0, 0, 0, 0.2);
          ">
            픽업존
          </div>
        </div>
      `
      
      // 픽업존 커스텀 오버레이
      const pickupOverlay = new window.kakao.maps.CustomOverlay({
        position: pickupPosition,
        content: pickupMarkerContent,
        map: map,
        yAnchor: 0
      })
      
      // 지도 로드 완료 후 컨테이너 스타일 조정
      setTimeout(() => {
        container.style.background = 'transparent'
      }, 100)
      
      console.log('배달 지도 초기화 완료')
      
    } catch (error) {
      console.error('배달 지도 초기화 실패:', error)
      const container = document.getElementById('delivery-map')
      if (container) {
        container.innerHTML = '<div style="display: flex; align-items: center; justify-content: center; height: 100%; color: #666; font-size: 16px;">배달 경로를 불러오는 중...</div>'
      }
    }
  }
  
  // 카카오맵 API 로드 확인 후 초기화
  const checkAndInitMap = () => {
    if (window.kakao && window.kakao.maps) {
      console.log('카카오맵 API 사용 가능')
      initDeliveryMap()
    } else {
      console.log('카카오맵 API 로드 대기 중...')
      setTimeout(checkAndInitMap, 500)
    }
  }
  
  // 즉시 시도
  checkAndInitMap()
  
  // 4초 후 배달 완료 모달 표시
  setTimeout(() => {
    console.log('배달 완료 모달 표시')
    showDeliveryCompleteModal.value = true
  }, 4000)
})
</script>

<style scoped>
.delivery-tracking-container {
  width: 100%;
  height: 100vh;
  height: 100dvh;
  background: #F9FAFB;
  display: flex;
  flex-direction: column;
  position: relative;
  overflow: hidden;
}

/* 지도 섹션 */
.map-section {
  flex: 1;
  position: relative;
  background: #F3F4F6;
  overflow: hidden;
}

.map-container {
  width: 100%;
  height: 100%;
  background: #E5E7EB;
}

/* 배달 상태 카드 */
.delivery-status-card {
  background: white;
  border-radius: 16px 16px 0 0;
  padding: 24px 32px;
  box-shadow: 0 -4px 20px rgba(0, 0, 0, 0.1);
  flex-shrink: 0;
  margin-top: -60px;
  position: relative;
  z-index: 10;
  min-height: 180px;
}

/* 상태 텍스트 */
.status-text {
  display: flex;
  align-items: center;
  justify-content: center;
  margin-bottom: 24px;
  position: relative;
  margin-top: 20px;
}

.status-left {
  display: flex;
  align-items: center;
  gap: 6px;
}

.lingki-name {
  font-size: 30px;
  font-weight: 700;
  color: #7C3AED;
}

.status-message {
  font-size: 22px;
  font-weight: 600;
  color: #1F2937;
}

.time-remaining {
  position: absolute;
  right: 0;
  top: 50%;
  transform: translateY(-50%);
  background: #7C3AED;
  padding: 8px 12px;
  border-radius: 8px;
  color: white;
  text-align: center;
  min-width: 60px;
}

.time-label {
  font-size: 10px;
  font-weight: 500;
  opacity: 0.9;
  margin-bottom: 2px;
}

.time-value {
  font-size: 14px;
  font-weight: 700;
}

/* 로봇 마스코트 */
.robot-mascot {
  display: flex;
  justify-content: center;
  margin-bottom: 24px;
}

.robot-image {
  width: 60px;
  height: 60px;
  object-fit: contain;
}

/* 타임라인 컨테이너 */
.timeline-container {
  position: relative;
  padding: 0 20px;
}

.timeline-line {
  position: relative;
  height: 2px;
  background: #E5E7EB;
  margin: 20px 0;
}

.timeline-progress {
  position: absolute;
  top: 0;
  left: 0;
  height: 100%;
  background: #7C3AED;
  width: 66%;
  transition: width 0.3s ease;
}

.timeline-markers {
  display: flex;
  justify-content: space-between;
  position: relative;
  margin-top: -10px;
}

.timeline-marker {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 8px;
}

.marker-dot {
  width: 12px;
  height: 12px;
  border-radius: 50%;
  border: 2px solid #E5E7EB;
  background: white;
  z-index: 2;
}

.timeline-marker.completed .marker-dot {
  background: #7C3AED;
  border-color: #7C3AED;
}

.timeline-marker.current .marker-dot {
  background: #7C3AED;
  border-color: #7C3AED;
  animation: pulse 2s infinite;
}

@keyframes pulse {
  0% {
    box-shadow: 0 0 0 0 rgba(124, 58, 237, 0.7);
  }
  70% {
    box-shadow: 0 0 0 10px rgba(124, 58, 237, 0);
  }
  100% {
    box-shadow: 0 0 0 0 rgba(124, 58, 237, 0);
  }
}

.marker-label {
  font-size: 12px;
  font-weight: 600;
  color: #6B7280;
  text-align: center;
}

.timeline-marker.completed .marker-label {
  color: #7C3AED;
}

.timeline-marker.current .marker-label {
  color: #7C3AED;
}

/* 모바일 반응형 */
@media (max-width: 480px) {
  .delivery-status-card {
    padding: 20px 24px;
    margin-top: -40px;
    min-height: 160px;
  }
  
  .lingki-name {
    font-size: 26px;
  }
  
  .status-message {
    font-size: 20px;
  }
  
  .time-remaining {
    padding: 6px 10px;
    min-width: 50px;
  }
  
  .time-label {
    font-size: 9px;
  }
  
  .time-value {
    font-size: 12px;
  }
  
  .robot-image {
    width: 50px;
    height: 50px;
  }
  
  .marker-label {
    font-size: 11px;
  }
}
</style>