<template>
  <div class="location-setting-container">
    <!-- 헤더 섹션 -->
    <div class="header-section">
      <h1 class="title">
        <span class="highlight">LiNKY</span>를 만날 위치를 설정하세요
      </h1>
      <p class="subtitle">핀 위치를 조정해주세요</p>
    </div>

    <!-- 지도 섹션 -->
    <div class="map-section">
      <div id="map" class="map-container"></div>
    </div>

         <!-- 주소 섹션 -->
     <div class="address-section">
       <div class="address-box">
         <div class="address-text">{{ currentAddress }}</div>
       </div>
       
       <button class="confirm-button" @click="confirmLocation">
         이 위치로 주소 설정
       </button>
     </div>
  </div>
</template>

<script setup>
import { onMounted, ref, watch } from 'vue'

const emit = defineEmits(['location-confirmed'])

// props로 얼굴 이미지 받기
const props = defineProps({
  faceImage: {
    type: String,
    default: ''
  }
})

const currentLocation = ref(null)
const currentAddress = ref('현재 위치를 가져오는 중...')
const map = ref(null)
const marker = ref(null)
const customOverlay = ref(null)
const capturedFaceImage = ref('') // 촬영된 얼굴 이미지

const confirmLocation = () => {
  console.log('위치 설정 완료')
  console.log('현재 위치:', currentLocation.value)
  console.log('현재 주소:', currentAddress.value)
  emit('location-confirmed', {
    location: currentLocation.value,
    address: currentAddress.value
  })
}

const getCurrentLocation = () => {
  return new Promise((resolve, reject) => {
    if (!navigator.geolocation) {
      reject(new Error('Geolocation이 지원되지 않습니다'))
      return
    }

    navigator.geolocation.getCurrentPosition(
      (position) => {
        const { latitude, longitude } = position.coords
        console.log('현재 위치:', latitude, longitude)
        resolve({ latitude, longitude })
      },
      (error) => {
        console.error('위치 가져오기 실패:', error)
        reject(error)
      },
      {
        enableHighAccuracy: true,
        timeout: 10000,
        maximumAge: 60000
      }
    )
  })
}

const getAddressFromCoords = async (latitude, longitude) => {
  try {
    // 카카오맵 API를 사용해서 좌표를 주소로 변환
    if (window.kakao && window.kakao.maps) {
      const geocoder = new window.kakao.maps.services.Geocoder()
      
      return new Promise((resolve, reject) => {
        geocoder.coord2Address(longitude, latitude, (result, status) => {
          if (status === window.kakao.maps.services.Status.OK) {
            const address = result[0].address.address_name
            console.log('주소 변환 성공:', address)
            resolve(address)
          } else {
            console.error('주소 변환 실패:', status)
            reject(new Error('주소 변환에 실패했습니다'))
          }
        })
      })
    } else {
      // 카카오맵 API가 없으면 좌표만 반환
      return `${latitude.toFixed(6)}, ${longitude.toFixed(6)}`
    }
  } catch (error) {
    console.error('주소 변환 오류:', error)
    return `${latitude.toFixed(6)}, ${longitude.toFixed(6)}`
  }
}

const updateLocationAndAddress = async (latitude, longitude) => {
  try {
    // 위치 업데이트
    currentLocation.value = { latitude, longitude }
    
    // 주소 업데이트
    const address = await getAddressFromCoords(latitude, longitude)
    currentAddress.value = address
    
    console.log('위치 및 주소 업데이트 완료:', { latitude, longitude }, address)
  } catch (error) {
    console.error('위치 및 주소 업데이트 실패:', error)
    currentAddress.value = `${latitude.toFixed(6)}, ${longitude.toFixed(6)}`
  }
}

const updateCustomMarker = (position) => {
  if (!customOverlay.value) return
  
  // 얼굴 이미지와 내 위치 텍스트 업데이트
  const faceContent = `
    <div style="position: relative;">
      <div style="
        position: absolute;
        top: 8px;
        left: 50%;
        transform: translateX(-50%);
        width: 32px;
        height: 32px;
        border-radius: 50%;
        border: 3px solid white;
        box-shadow: 0 2px 8px rgba(0, 0, 0, 0.1);
        overflow: hidden;
        background: #7C3AED;
        display: flex;
        align-items: center;
        justify-content: center;
        z-index: 1;
      ">
        ${capturedFaceImage.value ? 
          `<img src="${capturedFaceImage.value}" alt="얼굴" style="width: 100%; height: 100%; object-fit: cover;" />` : 
          '<span style="font-size: 16px; color: white;">👤</span>'
        }
      </div>
                          <!-- 내 위치 텍스트 -->
      <div style="
        position: absolute;
        top: 48px;
        left: 50%;
        transform: translateX(-50%);
        background: rgba(0, 0, 0, 0.8);
        color: white;
        padding: 4px 8px;
        border-radius: 6px;
        font-size: 12px;
        font-weight: 600;
        white-space: nowrap;
        z-index: 3;
        box-shadow: 0 2px 4px rgba(0, 0, 0, 0.2);
      ">
                              내 위치
      </div>
    </div>
  `
  
  customOverlay.value.setContent(faceContent)
  customOverlay.value.setPosition(position)
}

// 얼굴 이미지가 변경될 때 마커 업데이트
watch(() => props.faceImage, (newImage) => {
  if (newImage) {
    capturedFaceImage.value = newImage
    // 현재 마커 위치에서 업데이트
    if (customOverlay.value) {
      const position = customOverlay.value.getPosition()
      updateCustomMarker(position)
    }
  }
}, { immediate: true })

onMounted(async () => {
  // 현재 위치 가져오기
  try {
    console.log('현재 위치 가져오기 시작...')
    const location = await getCurrentLocation()
    currentLocation.value = location
    
    // 주소 변환
    const address = await getAddressFromCoords(location.latitude, location.longitude)
    currentAddress.value = address
    
    console.log('현재 위치 및 주소 설정 완료:', location, address)
  } catch (error) {
    console.error('현재 위치 가져오기 실패:', error)
    // 기본값으로 서울 시청 설정
    currentLocation.value = { latitude: 37.5665, longitude: 126.9780 }
    currentAddress.value = '종로구 동슬1길 4'
  }

  // 카카오맵 초기화 (모바일 대응)
  const initMap = () => {
    if (window.kakao && window.kakao.maps) {
      const container = document.getElementById('map')
      if (!container) return
      
      // 현재 위치 또는 기본 위치 사용
      const lat = currentLocation.value?.latitude || 37.5665
      const lng = currentLocation.value?.longitude || 126.9780
      
      const options = {
        center: new window.kakao.maps.LatLng(lat, lng),
        level: 4
      }
      
             try {
         const mapInstance = new window.kakao.maps.Map(container, options)
         map.value = mapInstance
         
                   // 드래그 가능한 마커 추가
          const markerPosition = new window.kakao.maps.LatLng(lat, lng)
          
          // 커스텀 마커 HTML 생성 (핀 이미지 + 실제 얼굴 이미지 + LiNKY 텍스트)
          const markerContent = `
            <div style="position: relative; display: inline-block; cursor: grab; user-select: none;" 
                 onmousedown="this.style.cursor='grabbing'" 
                 onmouseup="this.style.cursor='grab'"
                 onmouseleave="this.style.cursor='grab'">
              <div style="
                width: 40px;
                height: 40px;
                position: relative;
                z-index: 2;
                filter: drop-shadow(0 2px 4px rgba(0, 0, 0, 0.2));
                             ">
                 <img src="/src/assets/pin.png" alt="핀" style="width: 100%; height: 100%; object-fit: contain;" />
               </div>
              <div style="
                position: absolute;
                top: 35px;
                left: 50%;
                transform: translateX(-50%);
                width: 32px;
                height: 32px;
                border-radius: 50%;
                border: 3px solid white;
                box-shadow: 0 2px 8px rgba(0, 0, 0, 0.1);
                overflow: hidden;
                background: #7C3AED;
                display: flex;
                align-items: center;
                justify-content: center;
                z-index: 1;
              ">
                ${capturedFaceImage.value ? 
                  `<img src="${capturedFaceImage.value}" alt="얼굴" style="width: 100%; height: 100%; object-fit: cover;" />` : 
                  '<span style="font-size: 16px; color: white;">👤</span>'
                }
              </div>
                             <!-- 만날 위치 텍스트 -->
               <div style="
                 position: absolute;
                 top: 75px;
                 left: 50%;
                 transform: translateX(-50%);
                 background: rgba(0, 0, 0, 0.8);
                 color: white;
                 padding: 4px 8px;
                 border-radius: 6px;
                 font-size: 12px;
                 font-weight: 600;
                 white-space: nowrap;
                 z-index: 3;
                 box-shadow: 0 2px 4px rgba(0, 0, 0, 0.2);
               ">
                 만날 위치
               </div>
            </div>
          `
          
                    // 드래그 가능한 마커 생성 (커스텀 이미지 사용)
          const markerInstance = new window.kakao.maps.Marker({
            position: markerPosition,
            draggable: true,
            map: mapInstance,
            image: new window.kakao.maps.MarkerImage(
              '/src/assets/pin.png',
              new window.kakao.maps.Size(40, 40)
            )
          })
          
          marker.value = markerInstance
          
          // 마커 드래그 이벤트 리스너
          window.kakao.maps.event.addListener(markerInstance, 'dragend', function() {
            const position = markerInstance.getPosition()
            const lat = position.getLat()
            const lng = position.getLng()
            
            console.log('마커 드래그 완료:', lat, lng)
            overlayInstance.setPosition(position)
            updateLocationAndAddress(lat, lng)
          })
          
                                                                                       // 커스텀 오버레이로 얼굴 이미지와 내 위치 텍스트 표시
             const overlayInstance = new window.kakao.maps.CustomOverlay({
               position: markerPosition,
               content: `
                 <div style="position: relative;">
                   <div style="
                     position: absolute;
                     top: 8px;
                     left: 50%;
                     transform: translateX(-50%);
                     width: 32px;
                     height: 32px;
                     border-radius: 50%;
                     border: 3px solid white;
                     box-shadow: 0 2px 8px rgba(0, 0, 0, 0.1);
                     overflow: hidden;
                     background: #7C3AED;
                     display: flex;
                     align-items: center;
                     justify-content: center;
                     z-index: 1;
                   ">
                     ${capturedFaceImage.value ? 
                       `<img src="${capturedFaceImage.value}" alt="얼굴" style="width: 100%; height: 100%; object-fit: cover;" />` : 
                       '<span style="font-size: 16px; color: white;">👤</span>'
                     }
                   </div>
                   <!-- 내 위치 텍스트 -->
                   <div style="
                     position: absolute;
                     top: 48px;
                     left: 50%;
                     transform: translateX(-50%);
                     background: rgba(0, 0, 0, 0.8);
                     color: white;
                     padding: 4px 8px;
                     border-radius: 6px;
                     font-size: 12px;
                     font-weight: 600;
                     white-space: nowrap;
                     z-index: 3;
                     box-shadow: 0 2px 4px rgba(0, 0, 0, 0.2);
                   ">
                     내 위치
                   </div>
                 </div>
               `,
               map: mapInstance,
               yAnchor: 0
             })
          
          customOverlay.value = overlayInstance
         
                   // 지도 클릭 이벤트 리스너 (마커를 클릭한 위치로 이동)
          window.kakao.maps.event.addListener(mapInstance, 'click', function(mouseEvent) {
            const position = mouseEvent.latLng
            const lat = position.getLat()
            const lng = position.getLng()
            
            console.log('지도 클릭:', lat, lng)
            markerInstance.setPosition(position)
            overlayInstance.setPosition(position)
            updateLocationAndAddress(lat, lng)
          })
          

         
         // 지도 로드 완료 후 컨테이너 스타일 조정
         setTimeout(() => {
           container.style.background = 'transparent'
         }, 100)
        
      } catch (error) {
        console.error('카카오맵 초기화 실패:', error)
        // 폴백: 지도 대신 플레이스홀더 표시
        container.innerHTML = '<div style="display: flex; align-items: center; justify-content: center; height: 100%; color: #666; font-size: 16px;">지도를 불러오는 중...</div>'
      }
    } else {
      console.log('카카오맵 API가 로드되지 않았습니다.')
      // API가 로드되지 않은 경우 폴백
      const container = document.getElementById('map')
      if (container) {
        container.innerHTML = '<div style="display: flex; align-items: center; justify-content: center; height: 100%; color: #666; font-size: 16px;">지도를 불러오는 중...</div>'
      }
    }
  }

  // 즉시 시도
  initMap()
  
  // 약간의 지연 후 다시 시도 (모바일에서 API 로딩이 늦을 수 있음)
  setTimeout(initMap, 1000)
  
  // 추가 지연 시도
  setTimeout(initMap, 3000)
  
  // 카카오맵 API 로드 완료 이벤트 리스너
  window.addEventListener('kakao-maps-loaded', initMap)
  
  // 페이지 포커스 시 다시 시도 (모바일에서 백그라운드에서 포그라운드로 올 때)
  window.addEventListener('focus', () => {
    setTimeout(initMap, 500)
  })
})
</script>

<style scoped>
.location-setting-container {
  width: 100%;
  height: 100vh;
  height: 100dvh; /* Dynamic viewport height for mobile */
  background: #F9FAFB;
  display: flex;
  flex-direction: column;
  position: relative;
  overflow: hidden;
  box-sizing: border-box;
}

/* Header Section */
.header-section {
  padding: 20px 24px;
  background: white;
  border-bottom: 1px solid #E5E7EB;
}

.title {
  font-size: 18px;
  font-weight: 600;
  color: #1F2937;
  margin: 0;
  text-align: center;
}

.highlight {
  color: #7C3AED;
}

.subtitle {
  font-size: 14px;
  color: #6B7280;
  margin: 8px 0 0 0;
  text-align: center;
}

/* Map Section */
.map-section {
  flex: 1;
  position: relative;
  background: #F3F4F6;
  overflow: hidden;
  min-height: 0; /* Prevent flex overflow */
}

.map-container {
  width: 100%;
  height: 100%;
  background: #E5E7EB;
}



/* Address Section */
.address-section {
  background: white;
  padding: 20px 24px;
  border-top: 1px solid #E5E7EB;
  flex-shrink: 0; /* Prevent shrinking */
  position: relative;
  z-index: 1000;
}

.address-box {
  background: #F9FAFB;
  border: 1px solid #E5E7EB;
  border-radius: 12px;
  padding: 16px;
  margin-bottom: 16px;
}

.address-text {
  font-size: 16px;
  font-weight: 600;
  color: #1F2937;
  margin-bottom: 4px;
  text-align: center;
}

.address-link {
  font-size: 14px;
  color: #7C3AED;
  cursor: pointer;
  text-decoration: underline;
}

.confirm-button {
  width: 100%;
  height: 48px;
  background: #7C3AED;
  border: none;
  border-radius: 12px;
  color: white;
  font-size: 16px;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.3s ease;
}

.confirm-button:hover {
  background: #6D28D9;
  transform: translateY(-1px);
}

.confirm-button:active {
  transform: translateY(0);
}

/* Responsive Design */
@media (max-width: 480px) {
  .location-setting-container {
    height: 100vh;
    height: 100dvh;
    padding-bottom: env(safe-area-inset-bottom, 0px);
  }
  
  .header-section {
    padding: 16px 20px;
    flex-shrink: 0;
  }
  
  .title {
    font-size: 16px;
  }
  
  .map-section {
    flex: 1;
    min-height: 0;
  }
  
  .address-section {
    padding: 16px 20px;
    flex-shrink: 0;
    padding-bottom: calc(20px + env(safe-area-inset-bottom, 0px));
  }
  
  .address-box {
    padding: 12px;
  }
  
  .address-text {
    font-size: 15px;
  }
  
  .confirm-button {
    height: 44px;
    font-size: 15px;
  }
}
</style> 