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
         <div class="robot-body">
           <div class="robot-eyes">
             <div class="eye left"></div>
             <div class="eye right"></div>
           </div>
         </div>
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
      @show-compartment="$emit('show-compartment')"
    />
  </div>
</template>

<script setup>
import { onMounted, defineEmits, ref, defineProps } from 'vue'
import DeliveryCompleteModal from './11_DeliveryCompleteModal.vue'

const emit = defineEmits(['delivery-completed', 'show-compartment'])
const showDeliveryCompleteModal = ref(false)

// props로 배달 위치 정보 받기
const props = defineProps({
  deliveryLocation: {
    type: Object,
    default: () => ({ latitude: 37.5665, longitude: 126.9780 })
  },
  deliveryAddress: {
    type: String,
    default: '서울특별시 종로구'
  }
})

onMounted(() => {
  console.log('DeliveryTrackingScreen 마운트됨')
  console.log('배달 위치:', props.deliveryLocation)
  console.log('배달 주소:', props.deliveryAddress)
  
  // 카카오맵 초기화 (배달 경로 표시)
  const initDeliveryMap = () => {
    if (window.kakao && window.kakao.maps) {
      const container = document.getElementById('delivery-map')
      if (!container) return
      
      // 전달받은 위치 정보 사용
      const deliveryLat = props.deliveryLocation.latitude
      const deliveryLng = props.deliveryLocation.longitude
      
      console.log('지도 중심 설정:', deliveryLat, deliveryLng)
      
      const options = {
        center: new window.kakao.maps.LatLng(deliveryLat, deliveryLng),
        level: 3
      }
      
             try {
         const map = new window.kakao.maps.Map(container, options)
         
         // 목적지 마커 (사용자가 설정한 위치)
         const destPosition = new window.kakao.maps.LatLng(deliveryLat, deliveryLng)
         const destMarker = new window.kakao.maps.Marker({
           position: destPosition
         })
         destMarker.setMap(map)
        
        // 지도 로드 완료 후 컨테이너 스타일 조정
        setTimeout(() => {
          container.style.background = 'transparent'
        }, 100)
        
      } catch (error) {
        console.error('배달 지도 초기화 실패:', error)
        const container = document.getElementById('delivery-map')
        if (container) {
          container.innerHTML = '<div style="display: flex; align-items: center; justify-content: center; height: 100%; color: #666; font-size: 16px;">배달 경로를 불러오는 중...</div>'
        }
      }
    }
  }

  // 즉시 시도
  initDeliveryMap()
  
     // 지연 후 다시 시도
   setTimeout(initDeliveryMap, 1000)
   setTimeout(initDeliveryMap, 3000)
   
   // 4초 후 배달 완료 모달 표시 (비활성화)
   // setTimeout(() => {
   //   console.log('배달 완료 모달 표시')
   //   showDeliveryCompleteModal.value = true
   // }, 4000)
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
}

.time-label {
  font-size: 12px;
  color: #7C3AED;
  margin-bottom: 2px;
}

.time-value {
  font-size: 18px;
  font-weight: 700;
  color: #7C3AED;
}

/* 로봇 마스코트 */
.robot-mascot {
  display: flex;
  justify-content: center;
  margin-bottom: 24px;
  position: relative;
  z-index: 5;
  position: relative;
  top: 50px;
}

.robot-body {
  width: 36px;
  height: 36px;
  background: linear-gradient(135deg, #06B6D4, #0891B2);
  border-radius: 10px;
  display: flex;
  align-items: center;
  justify-content: center;
  position: relative;
  box-shadow: 0 4px 12px rgba(6, 182, 212, 0.3);
}

.robot-eyes {
  display: flex;
  gap: 3px;
}

.eye {
  width: 5px;
  height: 5px;
  background: #10B981;
  border-radius: 50%;
  box-shadow: 0 0 6px rgba(16, 185, 129, 0.8);
}

/* 타임라인 컨테이너 */
.timeline-container {
  margin-bottom: 30px;
}

.timeline-line {
  height: 2px;
  background: #E5E7EB;
  border-radius: 1px;
  margin-bottom: 50px;
  position: relative;
  margin-bottom: 30px;
  top: 50px;
  
}

.timeline-progress {
  height: 100%;
  background: #7C3AED;
  border-radius: 1px;
  width: 50%;
  transition: width 0.3s ease;
}

.timeline-markers {
  display: flex;
  justify-content: space-between;
  align-items: flex-start;
  position: relative;
  top: 15px;
}

.timeline-marker {
  display: flex;
  flex-direction: column;
  align-items: center;
  flex: 1;
}

.marker-dot {
  width: 12px;
  height: 12px;
  border-radius: 50%;
  margin-bottom: 8px;
  position: relative;
  z-index: 2;
  margin-top: 0px;
  border: 2px solid white;
  box-shadow: 0 0 0 2px #E5E7EB;
}

.timeline-marker.completed .marker-dot {
  background: #7C3AED;
}

.timeline-marker.current .marker-dot {
  background: #7C3AED;
}

.timeline-marker.pending .marker-dot {
  background: white;
  border: 2px solid #E5E7EB;
}

.marker-label {
  font-size: 12px;
  color: #6B7280;
  text-align: center;
  line-height: 1.2;
}

.timeline-marker.completed .marker-label,
.timeline-marker.current .marker-label {
  color: #7C3AED;
  font-weight: 600;
}

/* 반응형 디자인 */
@media (max-width: 480px) {
  .delivery-tracking-container {
    height: calc(100vh - 100px);
  }
  
  .delivery-status-card {
    padding: 20px;
    margin-bottom: -10px;
  }
  
  .status-text {
    margin-bottom: 16px;
  }
  
  .lingki-name {
    font-size: 30px;
  }
  
  .status-message {
    font-size: 18px;
  }
  
  .time-value {
    font-size: 16px;
  }
  
  .robot-body {
    width: 36px;
    height: 36px;
  }
  
  .timeline-markers {
    gap: 8px;
  }
  
  .marker-label {
    font-size: 11px;
  }
}
</style> 