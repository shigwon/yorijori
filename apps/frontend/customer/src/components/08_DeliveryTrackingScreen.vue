<template>
  <div class="delivery-tracking-container">
    <!-- 지도 섹션 -->
    <div v-if="!showStreaming" class="map-section">
      <div id="delivery-map" class="map-container"></div>
      
      <!-- 지도 위에 떠있는 실시간 스트리밍 버튼 -->
      <div class="floating-streaming-button">
        <button @click="toggleStreaming" class="streaming-button-floating">
          <span class="streaming-icon-floating">📹</span>
          <span class="streaming-text-floating">실시간 스트리밍</span>
        </button>
      </div>
    </div>

    <!-- 스트리밍 섹션 -->
    <div v-if="showStreaming" class="streaming-section">
      <!-- 스트리밍 헤더 -->
      <div class="streaming-header">
        <h3 class="streaming-title">
          <span class="robot-icon">🤖</span>
          {{ robotId }}번 로봇 실시간 스트리밍
        </h3>
        <button @click="toggleStreaming" class="back-to-map-button">
          <span class="back-icon">🗺️</span>
          <span class="back-text">지도로 돌아가기</span>
        </button>
      </div>
      
      <!-- 스트리밍 컨테이너 -->
      <div class="streaming-container">
        <!-- 로딩 상태 -->
        <div v-if="isLoading" class="loading-container">
          <div class="loading-spinner"></div>
          <p class="loading-text">SSE 연결 시도 중...</p>
          <p class="loading-detail">백엔드 서버: {{ API_BASE_URL || '프록시 사용' }}</p>
          <p class="loading-detail">로봇 ID: {{ robotId }}번</p>
        </div>
        
        <!-- 에러 상태 -->
        <div v-else-if="error" class="error-container">
          <div class="error-icon">⚠️</div>
          <p class="error-text">{{ error }}</p>
          <div class="error-actions">
            <button @click="retryConnection" class="retry-button" v-if="robotId && robotId !== ''">다시 시도</button>
            <button @click="goBackToMap" class="back-button">지도로 돌아가기</button>
          </div>
        </div>
        
        <!-- 스트리밍 화면 -->
        <div v-else class="streaming-display">
          <div class="streaming-image-container">
            <img 
              v-if="currentImage" 
              :src="currentImage" 
              alt="로봇 스트리밍" 
              class="streaming-image"
              @load="handleImageLoad"
              @error="handleImageError"
            />
            <div v-else class="no-image-placeholder">
              <span class="no-image-icon">📷</span>
              <p class="no-image-text">스트리밍 대기 중...</p>
              <p class="debug-info">디버그: currentImage = {{ currentImage ? '있음' : '없음' }}</p>
              <p class="debug-info">연결 상태: {{ isConnected ? '연결됨' : '연결 안됨' }}</p>
              <p class="debug-info">로딩 상태: {{ isLoading ? '로딩 중' : '로딩 완료' }}</p>
              <p class="debug-info">에러: {{ error || '없음' }}</p>
            </div>
          </div>
          
          <!-- 스트리밍 정보 -->
          <div class="streaming-info">
            <div class="info-row">
              <span class="info-label">로봇 ID:</span>
              <span class="info-value">{{ robotId }}번</span>
            </div>
            <div class="info-row">
              <span class="info-label">연결 상태:</span>
              <span class="info-value" :class="{ 'connected': isConnected }">
                {{ isConnected ? 'SSE 연결됨' : 'SSE 연결 중...' }}
              </span>
            </div>
            <div class="info-row">
              <span class="info-label">마지막 업데이트:</span>
              <span class="info-value">{{ lastUpdateTime }}</span>
            </div>
          </div>
        </div>
      </div>
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
      
      <!-- 배달 정보 섹션 -->
      <div v-if="robotId || sectionNum || orderCode" class="delivery-info-section">
        <div class="delivery-info-grid">
          <div v-if="robotId" class="delivery-info-item">
            <div class="info-icon">🤖</div>
            <div class="info-content">
              <div class="info-label">배달 로봇</div>
              <div class="info-value">{{ robotId }}번 로봇</div>
            </div>
          </div>
          
          <div v-if="sectionNum" class="delivery-info-item">
            <div class="info-icon">📦</div>
            <div class="info-content">
              <div class="info-label">음식함 번호</div>
              <div class="info-value">{{ sectionNum }}번 음식함</div>
            </div>
          </div>
          
          <div v-if="orderCode" class="delivery-info-item">
            <div class="info-icon">📋</div>
            <div class="info-content">
              <div class="info-label">주문번호</div>
              <div class="info-value">{{ orderCode }}</div>
            </div>
          </div>
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
import { onMounted, ref, onUnmounted } from 'vue'
import { useAppState } from '../composables/useAppState'
import DeliveryCompleteModal from './09_DeliveryCompleteModal.vue'

const { openFoodCompartment, deliveryLocation, deliveryAddress, capturedImage, robotId, sectionNum, orderCode } = useAppState()
const showDeliveryCompleteModal = ref(false)

// 스트리밍 관련 상태
const showStreaming = ref(false)
const isLoading = ref(false)
const error = ref(null)
const isConnected = ref(false)
const currentImage = ref(null)
const lastUpdateTime = ref('연결 대기 중...')
const eventSource = ref(null)

// API 기본 URL (프록시 설정 활용)
const API_BASE_URL = '' // 상대 경로 사용하여 프록시 활용

// 스트리밍 토글
const toggleStreaming = () => {
  if (showStreaming.value) {
    // 스트리밍 중지하고 지도로 돌아가기
    stopStreaming()
    showStreaming.value = false
  } else {
    // 스트리밍 화면으로 이동 (robotId 체크 제거)
    showStreaming.value = true
    
    // robotId가 있으면 스트리밍 시작, 없으면 연결 시도만
    if (robotId.value && robotId.value !== '') {
      startStreaming()
    } else {
      console.warn('⚠️ robotId가 설정되지 않음. 스트리밍 화면만 표시합니다.')
      // 로딩 상태 해제하고 에러 메시지 표시
      isLoading.value = false
      error.value = '로봇 ID가 설정되지 않았습니다.\n\n' +
                   'URL에 robotId 파라미터를 추가해주세요:\n' +
                   '예시: ?robotId=5&sectionNum=3&orderCode=ABC123\n\n' +
                   '이전 화면에서 사진 촬영이 완료되지 않았을 수 있습니다.'
    }
  }
}

// 백엔드 서버 연결 상태 확인
const checkBackendConnection = async () => {
  try {
    // 간단한 연결 테스트 (health 엔드포인트가 없을 수 있으므로)
    const controller = new AbortController()
    const timeoutId = setTimeout(() => controller.abort(), 3000)
    
    const response = await fetch('/api/v1/streaming/subscribe/test', { 
      method: 'HEAD', // HEAD 요청으로 연결만 확인
      signal: controller.signal
    })
    
    clearTimeout(timeoutId)
    return true // 연결 시도가 성공하면 true
  } catch (error) {
    if (error.name === 'AbortError') {
      console.error('백엔드 서버 연결 타임아웃')
    } else {
      console.error('백엔드 서버 연결 확인 실패:', error)
    }
    return false
  }
}

// 스트리밍 시작
const startStreaming = async () => {
  try {
    isLoading.value = true
    error.value = null
    
    console.log('SSE 스트리밍 시작, robotId:', robotId.value)
    
    // SSE 연결 생성
    const sseUrl = `${API_BASE_URL}/api/v1/streaming/subscribe/${robotId.value}`
    console.log('🔗 SSE 연결 시도:')
    console.log('- URL:', sseUrl)
    console.log('- robotId:', robotId.value)
    console.log('- API_BASE_URL:', API_BASE_URL)
    console.log('- 전체 URL:', window.location.origin + sseUrl)
    
    eventSource.value = new EventSource(sseUrl)
    
    // EventSource 생성 직후 상태 확인
    console.log('📡 EventSource 생성됨:')
    console.log('- readyState:', eventSource.value.readyState)
    console.log('- CONNECTING:', EventSource.CONNECTING)
    console.log('- OPEN:', EventSource.OPEN)
    console.log('- CLOSED:', EventSource.CLOSED)
    
    // 연결 타임아웃 설정 (10초 후 연결 실패로 처리)
    const connectionTimeout = setTimeout(() => {
      if (eventSource.value && eventSource.value.readyState === EventSource.CONNECTING) {
        console.error('⏰ SSE 연결 타임아웃 (10초)')
        eventSource.value.close()
        error.value = 'SSE 연결 타임아웃. 백엔드 서버를 확인해주세요.'
        isLoading.value = false
        isConnected.value = false
      }
    }, 10000)
    
    // 연결 성공 시 타임아웃 해제
    eventSource.value.onopen = () => {
      clearTimeout(connectionTimeout)
      console.log('🎉 SSE 연결 성공!')
      console.log('EventSource 상태:', eventSource.value.readyState)
      console.log('EventSource URL:', eventSource.value.url)
      console.log('📡 메시지 수신 대기 중...')
      console.log('⏰ 5초 후 메시지 수신 상태 확인 예정')
      
      isConnected.value = true
      isLoading.value = false
      error.value = null
      
      // 5초 후 메시지 수신 상태 확인
      setTimeout(() => {
        console.log('🔍 5초 후 메시지 수신 상태 확인:')
        console.log('- EventSource 상태:', eventSource.value?.readyState)
        console.log('- 연결 상태:', isConnected.value)
        console.log('- 메시지 수신 여부:', '아직 메시지 없음')
        console.log('- 백엔드에서 메시지를 보내고 있는지 확인 필요')
      }, 5000)
    }

    // SSE 메시지 수신
    eventSource.value.onmessage = (event) => {
      console.log('📨 SSE 기본 메시지 수신됨!')
      console.log('기본 메시지:', event.data)
    }

    // robotStreamingImage 이벤트 수신 (백엔드 API 명세서에 맞춤)
    eventSource.value.addEventListener('robotStreamingImage', (event) => {
      try {
        console.log('📨 robotStreamingImage 이벤트 수신됨!')
        console.log('이벤트 데이터:', event.data)
        console.log('메시지 타입:', typeof event.data)
        console.log('메시지 길이:', event.data.length)
        
        // 메시지가 너무 길면 일부만 표시
        if (event.data.length > 100) {
          console.log('메시지 미리보기:', event.data.substring(0, 100) + '...')
        } else {
          console.log('전체 메시지:', event.data)
        }
        
        // 백엔드에서 보내는 형식에 맞게 처리
        if (event.data.startsWith('WAITING')) {
          console.log('⏳ 대기 상태 메시지:', event.data)
          // 대기 상태 처리
          return
        }
        
        // Base64 이미지 데이터 처리 (image/jpg;base64, 형식)
        if (event.data.startsWith('image/jpg;base64,') || event.data.startsWith('image/jpeg;base64,')) {
          console.log('🖼️ Base64 이미지 데이터 감지됨!')
          
          // Base64 데이터 추출 (헤더 제거)
          const base64Data = event.data.replace(/^image\/[^;]+;base64,/, '')
          console.log('Base64 데이터 길이:', base64Data.length)
          
          try {
            // Base64 디코딩
            const byteCharacters = atob(base64Data)
            console.log('Base64 디코딩 완료, 바이트 수:', byteCharacters.length)
            
            const byteNumbers = new Array(byteCharacters.length)
            for (let i = 0; i < byteCharacters.length; i++) {
              byteNumbers[i] = byteCharacters.charCodeAt(i)
            }
            const byteArray = new Uint8Array(byteNumbers)
            console.log('바이트 배열 생성 완료:', byteArray.length)
            
            // JPEG Blob 생성
            const blob = new Blob([byteArray], { type: 'image/jpeg' })
            console.log('Blob 생성 완료:', blob)
            console.log('Blob 크기:', blob.size)
            console.log('Blob 타입:', blob.type)
            
            // 이전 이미지 URL 해제
            if (currentImage.value) {
              console.log('이전 이미지 URL 해제:', currentImage.value)
              URL.revokeObjectURL(currentImage.value)
            }
            
            // 새 이미지 URL 생성
            const imageUrl = URL.createObjectURL(blob)
            console.log('새 이미지 URL 생성:', imageUrl)
            
            // 이미지 설정
            currentImage.value = imageUrl
            console.log('새 이미지 설정 완료:', currentImage.value)
            
            // 상태 업데이트
            lastUpdateTime.value = new Date().toLocaleTimeString('ko-KR')
            isConnected.value = true
            isLoading.value = false
            error.value = null
            
            console.log('🎉 이미지 업데이트 완료:', lastUpdateTime.value)
            console.log('현재 이미지 상태:', currentImage.value ? '있음' : '없음')
            
          } catch (decodeError) {
            console.error('❌ Base64 디코딩 실패:', decodeError)
            error.value = '이미지 디코딩에 실패했습니다: ' + decodeError.message
            isLoading.value = false
          }
          return
        }
        
        // JSON 파싱 시도 (기존 로직)
        let data
        try {
          data = JSON.parse(event.data)
          console.log('✅ JSON 파싱 성공!')
        } catch (parseError) {
          console.log('⚠️ JSON 파싱 실패, 일반 텍스트로 처리:', event.data)
          // 일반 텍스트인 경우 기본 처리
          data = { type: 'text', content: event.data }
        }
        
        console.log('파싱된 데이터:', data)
        console.log('데이터 타입:', data.type)
        
        if (data.type === 'image') {
          console.log('🖼️ JSON 이미지 타입 확인됨, 처리 시작...')
          
          // Base64 이미지 데이터를 Blob으로 변환
          console.log('Base64 디코딩 시작...')
          const byteCharacters = atob(data.image)
          console.log('Base64 디코딩 완료, 바이트 수:', byteCharacters.length)
          
          const byteNumbers = new Array(byteCharacters.length)
          for (let i = 0; i < byteCharacters.length; i++) {
            byteNumbers[i] = byteCharacters.charCodeAt(i)
          }
          const byteArray = new Uint8Array(byteNumbers)
          console.log('바이트 배열 생성 완료:', byteArray.length)
          
          const blob = new Blob([byteArray], { type: 'image/jpeg' })
          console.log('Blob 생성 완료:', blob)
          console.log('Blob 크기:', blob.size)
          console.log('Blob 타입:', blob.type)
          
          const imageUrl = URL.createObjectURL(blob)
          console.log('이미지 URL 생성:', imageUrl)

          // 이전 이미지 URL 해제
          if (currentImage.value) {
            console.log('이전 이미지 URL 해제:', currentImage.value)
            URL.revokeObjectURL(currentImage.value)
          }

          console.log('새 이미지 설정 전 currentImage.value:', currentImage.value)
          currentImage.value = imageUrl
          console.log('새 이미지 설정 후 currentImage.value:', currentImage.value)
          
          lastUpdateTime.value = new Date().toLocaleTimeString('ko-KR')
          isConnected.value = true
          console.log('🎉 이미지 업데이트 완료:', lastUpdateTime.value)
          console.log('현재 이미지 상태:', currentImage.value ? '있음' : '없음')
        } else if (data.type === 'text') {
          console.log('📝 텍스트 메시지:', data.content)
          // 텍스트 메시지 처리
        } else {
          console.log('❓ 알 수 없는 타입:', data.type)
          console.log('받은 데이터:', data)
        }
      } catch (err) {
        console.error('❌ robotStreamingImage 이벤트 처리 실패:', err)
        console.error('원본 데이터:', event.data)
        console.error('에러 스택:', err.stack)
        error.value = '메시지 처리에 실패했습니다: ' + err.message
        isLoading.value = false
      }
    })
    
    // SSE 연결 에러
    eventSource.value.onerror = (error) => {
      clearTimeout(connectionTimeout) // 타임아웃 해제
      console.error('❌ SSE 연결 에러 발생:')
      console.error('- 에러 타입:', error.type)
      console.error('- 에러 타겟:', error.target)
      console.error('- EventSource 상태:', error.target?.readyState)
      console.error('- EventSource URL:', error.target?.url)
      console.error('- 전체 에러 객체:', error)
      
      isConnected.value = false
      
      // 더 구체적인 에러 메시지
      if (error.target && error.target.readyState === EventSource.CONNECTING) {
        error.value = '백엔드 서버에 연결할 수 없습니다. 서버가 실행 중인지 확인해주세요.'
      } else if (error.target && error.target.readyState === EventSource.CLOSED) {
        error.value = 'SSE 연결이 종료되었습니다. 다시 시도해주세요.'
      } else {
        error.value = '스트리밍 연결에 실패했습니다. 백엔드 서버를 확인해주세요.'
      }
      
      // 백엔드 서버 실행 안내
      console.warn('💡 백엔드 서버 실행 방법:')
      console.warn('1. backend 디렉토리로 이동')
      console.warn('2. ./gradlew bootRun 실행')
      console.warn('3. 또는 IDE에서 Spring Boot 애플리케이션 실행')
      console.warn('4. 포트 8080에서 실행되는지 확인')
      
      isLoading.value = false
      
      // 연결 재시도 (서버가 시작될 때까지)
      setTimeout(() => {
        if (showStreaming.value && !isConnected.value) {
          console.log('🔄 SSE 연결 재시도...')
          startStreaming()
        }
      }, 5000) // 5초 후 재시도
    }
    
    isLoading.value = false
  } catch (err) {
    console.error('스트리밍 시작 실패:', err)
    error.value = '스트리밍을 시작할 수 없습니다: ' + err.message
    isLoading.value = false
    isConnected.value = false
  }
}

// 스트리밍 중지
const stopStreaming = () => {
  if (eventSource.value) {
    eventSource.value.close()
    eventSource.value = null
  }
  isConnected.value = false
}

// 연결 재시도
const retryConnection = () => {
  error.value = null
  startStreaming()
}

// 지도로 돌아가기
const goBackToMap = () => {
  showStreaming.value = false
  // 스트리밍 중지
  stopStreaming()
}

// 이미지 로드 성공 핸들러
const handleImageLoad = () => {
  console.log('🖼️ 이미지 로드 완료:', currentImage.value)
}

// 이미지 로드 실패 핸들러
const handleImageError = (event) => {
  console.error('❌ 이미지 로드 실패:', event.target.src)
  error.value = '이미지를 불러올 수 없습니다.'
  isLoading.value = false
  isConnected.value = false
}

onMounted(() => {
  console.log('DeliveryTrackingScreen 마운트됨')
  console.log('useAppState 배달 위치:', deliveryLocation.value)
  console.log('useAppState 배달 주소:', deliveryAddress.value)
  console.log('useAppState 사용자 사진:', capturedImage.value ? '있음' : '없음')
  console.log('useAppState robotId:', robotId.value)
  console.log('useAppState sectionNum:', sectionNum.value)
  console.log('useAppState orderCode:', orderCode.value)
  
  // 실시간 스트리밍 버튼 디버깅
  console.log('🔍 실시간 스트리밍 버튼 상태 확인:')
  console.log('- showStreaming:', showStreaming.value)
  console.log('- robotId 존재 여부:', !!robotId.value)
  console.log('- robotId 값:', robotId.value)
  
  // robotId가 없으면 경고
  if (!robotId.value || robotId.value === '') {
    console.warn('⚠️ robotId가 설정되지 않음. URL에 ?robotId=숫자 파라미터를 추가해주세요.')
    console.warn('예시: /customer/delivery-tracking?robotId=5&sectionNum=3&orderCode=ABC123')
  }
  
  // 실시간 스트리밍 버튼 요소 확인
  setTimeout(() => {
    const streamingButton = document.querySelector('.floating-streaming-button')
    const streamingButtonInner = document.querySelector('.streaming-button-floating')
    console.log('🔍 실시간 스트리밍 버튼 DOM 요소 확인:')
    console.log('- .floating-streaming-button:', streamingButton)
    console.log('- .streaming-button-floating:', streamingButtonInner)
    
    if (streamingButton) {
      console.log('✅ 실시간 스트리밍 버튼 컨테이너 발견')
      console.log('- computed styles:', window.getComputedStyle(streamingButton))
    } else {
      console.error('❌ 실시간 스트리밍 버튼 컨테이너를 찾을 수 없음')
    }
    
    if (streamingButtonInner) {
      console.log('✅ 실시간 스트리밍 버튼 내부 요소 발견')
      console.log('- computed styles:', window.getComputedStyle(streamingButtonInner))
    } else {
      console.error('❌ 실시간 스트리밍 버튼 내부 요소를 찾을 수 없음')
    }
  }, 1000)
  
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
      console.log('배달 지도에서 사용자 이미지:', userImage ? '있음' : '없음')
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
  

  // 5초 후 자동으로 배달완료 모달 표시 (테스트용 - 주석 처리)
  // setTimeout(() => {
  //   console.log('배달 완료 모달 표시')
  //   showDeliveryCompleteModal.value = true
  // }, 5000)
})

// 컴포넌트 언마운트 시 정리
onUnmounted(() => {
  stopStreaming()
  if (currentImage.value) {
    URL.revokeObjectURL(currentImage.value)
  }
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
  overflow: visible;
  min-height: 400px;
}

.map-container {
  width: 100%;
  height: 100%;
  background: #E5E7EB;
}

/* 지도 위에 떠있는 실시간 스트리밍 버튼 */
.floating-streaming-button {
  position: absolute;
  top: 20px;
  left: 20px;
  z-index: 1000;
  pointer-events: auto;
}

.streaming-button-floating {
  background: rgba(124, 58, 237, 0.95);
  color: white;
  padding: 12px 20px;
  border-radius: 25px;
  border: none;
  cursor: pointer;
  display: flex;
  align-items: center;
  gap: 8px;
  font-size: 14px;
  font-weight: 600;
  box-shadow: 0 4px 20px rgba(0, 0, 0, 0.3);
  transition: all 0.3s ease;
  backdrop-filter: blur(10px);
  border: 1px solid rgba(255, 255, 255, 0.2);
  min-width: 140px;
  justify-content: center;
}

.streaming-button-floating:hover {
  background: rgba(109, 40, 217, 0.95);
  transform: translateY(-2px);
  box-shadow: 0 6px 25px rgba(0, 0, 0, 0.4);
}

.streaming-icon-floating {
  font-size: 18px;
}

.streaming-text-floating {
  font-size: 14px;
  white-space: nowrap;
}

/* 스트리밍 섹션 */
.streaming-section {
  flex: 1;
  position: relative;
  background: #F3F4F6;
  overflow: hidden;
  display: flex;
  flex-direction: column;
}

.streaming-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 20px 32px;
  background: #7C3AED;
  color: white;
  box-shadow: 0 4px 10px rgba(0, 0, 0, 0.1);
}

.streaming-title {
  display: flex;
  align-items: center;
  gap: 8px;
  font-size: 24px;
  font-weight: 700;
}

.robot-icon {
  font-size: 28px;
}

.back-to-map-button {
  background: rgba(255, 255, 255, 0.2);
  color: white;
  padding: 8px 12px;
  border-radius: 8px;
  border: none;
  cursor: pointer;
  display: flex;
  align-items: center;
  gap: 6px;
  font-size: 14px;
  font-weight: 600;
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.2);
  transition: background-color 0.3s ease;
}

.back-to-map-button:hover {
  background: rgba(255, 255, 255, 0.3);
}

.back-icon {
  font-size: 18px;
}

.back-text {
  font-size: 14px;
}

.streaming-container {
  flex: 1;
  display: flex;
  justify-content: center;
  align-items: center;
  background: #F3F4F6;
  padding: 20px;
}

.loading-container {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  gap: 15px;
  padding: 30px;
  background: white;
  border-radius: 12px;
  box-shadow: 0 4px 15px rgba(0, 0, 0, 0.1);
}

.loading-spinner {
  border: 4px solid #f3f3f3;
  border-top: 4px solid #7C3AED;
  border-radius: 50%;
  width: 40px;
  height: 40px;
  animation: spin 1s linear infinite;
}

@keyframes spin {
  0% { transform: rotate(0deg); }
  100% { transform: rotate(360deg); }
}

.loading-text {
  font-size: 16px;
  color: #6B7280;
  font-weight: 500;
}

.loading-detail {
  font-size: 12px;
  color: #9CA3AF;
  margin-top: 5px;
}

.error-container {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  gap: 15px;
  padding: 30px;
  background: white;
  border-radius: 12px;
  box-shadow: 0 4px 15px rgba(0, 0, 0, 0.1);
}

.error-icon {
  font-size: 40px;
  color: #EF4444;
}

.error-text {
  font-size: 16px;
  color: #6B7280;
  font-weight: 500;
  text-align: center;
}

.error-actions {
  display: flex;
  gap: 10px;
}

.retry-button,
.back-button {
  background: #7C3AED;
  color: white;
  padding: 10px 20px;
  border-radius: 8px;
  border: none;
  cursor: pointer;
  font-size: 14px;
  font-weight: 600;
  transition: background-color 0.3s ease;
}

.retry-button:hover,
.back-button:hover {
  background: #6D28D9;
}

.back-button {
  background: #6B7280;
}

.back-button:hover {
  background: #4B5563;
}

.streaming-display {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  gap: 20px;
  padding: 20px;
  background: white;
  border-radius: 12px;
  box-shadow: 0 4px 15px rgba(0, 0, 0, 0.1);
}

.streaming-image-container {
  width: 100%;
  max-width: 600px;
  aspect-ratio: 16 / 9;
  background: #E0E0E0;
  border-radius: 8px;
  overflow: hidden;
  display: flex;
  align-items: center;
  justify-content: center;
  box-shadow: 0 4px 10px rgba(0, 0, 0, 0.1);
  position: relative;
}

.streaming-image {
  width: 100%;
  height: 100%;
  object-fit: contain;
  background: #000;
  border-radius: 8px;
  /* 이미지 로딩 상태 개선 */
  transition: opacity 0.3s ease;
}

.streaming-image:not([src]) {
  opacity: 0;
}

.streaming-image[src] {
  opacity: 1;
}

.no-image-placeholder {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  gap: 10px;
  color: #6B7280;
  font-size: 16px;
}

.no-image-icon {
  font-size: 40px;
}

.streaming-info {
  display: flex;
  flex-direction: column;
  gap: 8px;
  padding: 0 20px;
}

.info-row {
  display: flex;
  justify-content: space-between;
  align-items: center;
  font-size: 14px;
  color: #4B5563;
}

.info-label {
  font-weight: 500;
  opacity: 0.7;
}

.info-value {
  font-weight: 600;
  color: #1F2937;
}

.info-value.connected {
  color: #10B981; /* 연결됨 색상 */
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

/* 배달 정보 섹션 */
.delivery-info-section {
  margin-top: 20px;
  padding: 16px 0;
  border-top: 1px solid #E5E7EB;
  border-bottom: 1px solid #E5E7EB;
}

.delivery-info-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(120px, 1fr));
  gap: 12px;
  justify-items: center;
}

.delivery-info-item {
  display: flex;
  align-items: center;
  gap: 8px;
  color: #4B5563;
  font-size: 14px;
  font-weight: 500;
}

.info-icon {
  font-size: 20px;
}

.info-content {
  display: flex;
  flex-direction: column;
}

.info-label {
  font-size: 10px;
  opacity: 0.7;
  margin-bottom: 2px;
}

.info-value {
  font-size: 14px;
  font-weight: 600;
  color: #1F2937;
}

/* 실시간 스트리밍 버튼 */
.streaming-button-container {
  display: flex;
  justify-content: center;
  margin-top: 20px;
}

.streaming-button {
  background: #7C3AED;
  color: white;
  padding: 12px 24px;
  border-radius: 12px;
  border: none;
  cursor: pointer;
  display: flex;
  align-items: center;
  gap: 8px;
  font-size: 16px;
  font-weight: 600;
  box-shadow: 0 4px 12px rgba(124, 58, 237, 0.2);
  transition: background-color 0.3s ease;
}

.streaming-button:hover {
  background: #6D28D9;
}

.streaming-icon {
  font-size: 20px;
}

.streaming-text {
  font-size: 16px;
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
  transform: translateY(17px);
}

.timeline-progress {
  position: absolute;
  top: 0;
  left: 0;
  height: 100%;
  background: #7C3AED;
  width: 51%;
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
  
  .delivery-info-section {
    margin-top: 16px;
    padding: 12px 0;
  }
  
  .delivery-info-grid {
    gap: 8px;
  }
  
  .delivery-info-item {
    font-size: 12px;
  }
  
  .info-icon {
    font-size: 18px;
  }
  
  .info-label {
    font-size: 9px;
  }
  
  .info-value {
    font-size: 12px;
  }
  
  /* 모바일에서 지도 위 스트리밍 버튼 */
  .floating-streaming-button {
    top: 15px;
    left: 15px;
  }
  
  .streaming-button-floating {
    padding: 10px 16px;
    font-size: 12px;
  }
  
  .streaming-icon-floating {
    font-size: 16px;
  }
  
  .streaming-text-floating {
    font-size: 12px;
  }
  
  /* 모바일에서 스트리밍 섹션 */
  .streaming-header {
    padding: 16px 20px;
  }
  
  .streaming-title {
    font-size: 20px;
  }
  
  .robot-icon {
    font-size: 24px;
  }
  
  .back-to-map-button {
    padding: 6px 10px;
    font-size: 12px;
  }
  
  .back-icon {
    font-size: 16px;
  }
  
  .back-text {
    font-size: 12px;
  }
  
  .streaming-container {
    padding: 16px;
  }
  
  .streaming-image-container {
    max-width: 100%;
  }
  
  .loading-container,
  .error-container,
  .streaming-display {
    padding: 20px;
  }
  
  .loading-spinner {
    width: 32px;
    height: 32px;
  }
  
  .loading-text,
  .error-text {
    font-size: 14px;
  }
  
  .error-icon,
  .no-image-icon {
    font-size: 32px;
  }
  
  .retry-button,
  .back-button {
    padding: 8px 16px;
    font-size: 12px;
  }
}

.no-image-text {
  margin: 0;
  font-size: 16px;
  font-weight: 500;
}

.debug-info {
  margin: 5px 0 0 0;
  font-size: 12px;
  color: #9CA3AF;
  font-family: monospace;
}
</style>