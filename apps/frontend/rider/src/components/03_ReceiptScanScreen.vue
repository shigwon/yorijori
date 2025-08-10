<template>
  <div class="receipt-scan-container">
    <!-- 헤더 섹션 -->
    <div class="header-section">
      <div class="scan-title-text">영수증 이미지를 촬영해주세요</div>
    </div>

    <!-- 카메라 섹션 -->
    <div class="camera-section">
      <div class="camera-preview">
        <video ref="videoElement" autoplay playsinline class="camera-video" v-if="!cameraError"></video>
        <div class="camera-placeholder" v-if="cameraError">
          <div class="placeholder-content">
            <div class="placeholder-icon">
              <svg width="64" height="64" viewBox="0 0 64 64" fill="none">
                <rect x="8" y="16" width="48" height="32" rx="4" fill="#374151" stroke="#6B7280" stroke-width="2"/>
                <circle cx="32" cy="32" r="12" fill="#6B7280"/>
                <circle cx="32" cy="32" r="6" fill="#9CA3AF"/>
                <rect x="24" y="12" width="16" height="8" rx="2" fill="#6B7280"/>
              </svg>
            </div>
                         <p class="placeholder-text">{{ cameraStatus }}</p>
             <p class="placeholder-subtext">아래 버튼을 눌러 카메라 권한을 허용해주세요</p>
            <button class="camera-permission-btn" @click="requestCameraPermission">
              카메라 권한 허용
            </button>
          </div>
        </div>
        <div class="scan-frame">
          <div class="corner tl"></div>
          <div class="corner tr"></div>
          <div class="corner bl"></div>
          <div class="corner br"></div>
        </div>
      </div>
    </div>

    <!-- 촬영 버튼 섹션 -->
    <div class="capture-section">
      <button class="capture-button" @click="captureImage" :disabled="isCapturing">
        <div v-if="isCapturing" class="loading-spinner"></div>
      </button>
    </div>

    <!-- 캡처용 캔버스 -->
    <canvas ref="canvas" style="display: none;"></canvas>
  </div>
</template>

<script setup>
import { ref, onMounted } from 'vue'
import { useAppState } from '../composables/useAppState'

const { goToScanConfirmModal, goToLoadingModal, setProgressPercent, receiptData } = useAppState()

const videoElement = ref(null)
const canvas = ref(null)
const cameraError = ref(false)
const cameraStatus = ref('카메라 로딩 중...')
const isCapturing = ref(false)

onMounted(() => {
  setProgressPercent(40)
  startCamera()
})

const startCamera = async () => {
  console.log('카메라 시작 시도...')
  console.log('브라우저:', navigator.userAgent)
  console.log('HTTPS 여부:', window.location.protocol === 'https:')
  
  cameraStatus.value = '카메라 초기화 중...'
  
  // HTTPS 확인 (ngrok 사용 시 필요)
  if (window.location.protocol !== 'https:' && window.location.hostname !== 'localhost') {
    console.log('HTTPS가 필요합니다. ngrok을 사용해주세요.')
    cameraStatus.value = 'HTTPS가 필요합니다. ngrok을 사용해주세요.'
    cameraError.value = true
    return
  }
  
  // getUserMedia 지원 확인
  if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
    console.log('getUserMedia 지원 안됨')
    cameraStatus.value = '이 브라우저는 카메라를 지원하지 않습니다.'
    cameraError.value = true
    return
  }

  try {
    // 휴대폰 최적화 설정
    const constraints = {
      video: {
        width: { min: 320, ideal: 1280, max: 1920 },
        height: { min: 240, ideal: 720, max: 1080 },
        facingMode: { ideal: 'environment' }, // 후면 카메라 우선
        aspectRatio: { ideal: 4/3 }
      },
      audio: false
    }
    
    cameraStatus.value = '카메라 스트림 요청 중...'
    console.log('카메라 스트림 요청 중...')
    const stream = await navigator.mediaDevices.getUserMedia(constraints)
    console.log('카메라 스트림 성공:', stream)
    
    if (videoElement.value) {
      videoElement.value.srcObject = stream
      
      // 비디오 로드 이벤트
      videoElement.value.onloadedmetadata = () => {
        console.log('비디오 메타데이터 로드됨')
        cameraStatus.value = '카메라 준비 완료'
        cameraError.value = false
      }
      
      videoElement.value.oncanplay = () => {
        console.log('비디오 재생 가능')
        cameraStatus.value = '카메라 준비 완료'
        cameraError.value = false
      }
      
      videoElement.value.onplay = () => {
        console.log('비디오 재생 시작')
        cameraStatus.value = '카메라 실행 중'
        cameraError.value = false
      }
      
      videoElement.value.onerror = (e) => {
        console.log('비디오 오류:', e)
        cameraStatus.value = '카메라 오류 발생'
        cameraError.value = true
      }
      
      // 휴대폰에서 안정적인 재생을 위한 지연
      setTimeout(async () => {
        if (videoElement.value) {
          try {
            await videoElement.value.play()
            console.log('비디오 재생 성공')
          } catch (err) {
            console.log('재생 실패:', err)
            // 사용자 상호작용 후 재시도
            if (err.name === 'NotAllowedError') {
              console.log('사용자 상호작용 필요')
            }
          }
        }
      }, 500)
      
    } else {
      console.log('videoElement가 없음')
      cameraStatus.value = '비디오 요소를 찾을 수 없습니다'
      cameraError.value = true
    }
  } catch (err) {
    console.error('카메라 접근 실패:', err)
    
    if (err.name === 'NotAllowedError') {
      console.log('카메라 권한이 거부되었습니다. 브라우저 설정에서 카메라 권한을 허용해주세요.')
    } else if (err.name === 'NotFoundError') {
      console.log('카메라를 찾을 수 없습니다.')
    } else if (err.name === 'NotReadableError') {
      console.log('카메라가 다른 앱에서 사용 중입니다.')
    } else if (err.name === 'OverconstrainedError') {
      console.log('지원하지 않는 카메라 설정입니다.')
      // 더 간단한 설정으로 재시도
      try {
        const simpleStream = await navigator.mediaDevices.getUserMedia({ 
          video: { facingMode: 'environment' },
          audio: false 
        })
        if (videoElement.value) {
          videoElement.value.srcObject = simpleStream
          await videoElement.value.play()
          cameraError.value = false
        }
      } catch (simpleErr) {
        console.log('간단한 설정도 실패:', simpleErr)
        cameraError.value = true
      }
    }
    
    cameraError.value = true
  }
}

const requestCameraPermission = async () => {
  console.log('카메라 권한 요청 중...')
  cameraStatus.value = '카메라 권한 요청 중...'
  cameraError.value = false
  
  try {
    // 가장 기본적인 설정으로 권한 요청
    const stream = await navigator.mediaDevices.getUserMedia({ 
      video: true,
      audio: false 
    })
    
    console.log('스트림 획득 성공:', stream)
    
    if (videoElement.value) {
      videoElement.value.srcObject = stream
      
      // 비디오 이벤트 리스너 추가
      videoElement.value.onloadedmetadata = () => {
        console.log('비디오 메타데이터 로드됨')
        cameraStatus.value = '카메라 준비 완료'
        cameraError.value = false
      }
      
      videoElement.value.oncanplay = () => {
        console.log('비디오 재생 가능')
        cameraStatus.value = '카메라 준비 완료'
        cameraError.value = false
      }
      
      videoElement.value.onplay = () => {
        console.log('비디오 재생 시작')
        cameraStatus.value = '카메라 실행 중'
        cameraError.value = false
      }
      
      videoElement.value.onerror = (e) => {
        console.log('비디오 오류:', e)
        cameraStatus.value = '카메라 오류 발생'
        cameraError.value = true
      }
      
      // 재생 시도
      try {
        await videoElement.value.play()
        console.log('카메라 권한 허용됨 - 재생 성공')
        cameraStatus.value = '카메라 실행 중'
      } catch (playErr) {
        console.log('재생 실패:', playErr)
        // 재생 실패해도 스트림은 연결됨
        cameraStatus.value = '카메라 연결됨 (재생 대기 중)'
        cameraError.value = false
      }
    } else {
      console.log('videoElement가 없음')
      cameraStatus.value = '비디오 요소를 찾을 수 없습니다'
      cameraError.value = true
    }
  } catch (err) {
    console.error('카메라 권한 요청 실패:', err)
    console.log('에러 이름:', err.name)
    console.log('에러 메시지:', err.message)
    
    if (err.name === 'NotAllowedError') {
      cameraStatus.value = '카메라 권한이 거부되었습니다'
      alert('카메라 권한이 거부되었습니다. 브라우저 설정에서 카메라 권한을 허용해주세요.')
    } else if (err.name === 'NotFoundError') {
      cameraStatus.value = '카메라를 찾을 수 없습니다'
      alert('카메라를 찾을 수 없습니다.')
    } else if (err.name === 'NotReadableError') {
      cameraStatus.value = '카메라가 다른 앱에서 사용 중입니다'
      alert('카메라가 다른 앱에서 사용 중입니다.')
    } else {
      cameraStatus.value = '카메라 접근 실패: ' + err.message
      alert('카메라 접근에 실패했습니다: ' + err.message)
    }
    
    cameraError.value = true
  }
}

const captureImage = async () => {
  // 이미 캡처 중이면 중복 실행 방지
  if (isCapturing.value) {
    return
  }
  
  isCapturing.value = true
  
  try {
    if (cameraError.value) {
      // 카메라 오류 시에도 로딩 모달로 이동
      goToLoadingModal()
      return
    }
    
    const video = videoElement.value
    if (video && video.videoWidth > 0) {
      const ctx = canvas.value.getContext('2d')
      canvas.value.width = video.videoWidth
      canvas.value.height = video.videoHeight
      ctx.drawImage(video, 0, 0, video.videoWidth, video.videoHeight)
      
      // Canvas를 Base64로 변환
      const base64Image = canvas.value.toDataURL('image/jpeg', 0.9)
      const base64Only = base64Image.split(',')[1]
      console.log('Base64 이미지 생성 완료')
      
      // 로딩 모달로 이동
      goToLoadingModal()
      
      // 백그라운드에서 API 호출
      processImageAsync(base64Only)
      
    } else {
      // 비디오가 준비되지 않은 경우
      goToLoadingModal()
    }
  } catch (error) {
    console.error('이미지 캡처 실패:', error)
    goToLoadingModal()
  } finally {
    isCapturing.value = false
  }
}

// 백그라운드에서 이미지 처리
const processImageAsync = async (base64Image) => {
  try {
    // JSON 데이터로 전송
    const requestData = {
      image: base64Image
    }

    console.log('API 요청 시작...')
    
    // API 호출 (5초 타임아웃 설정)
    const controller = new AbortController()
    const timeoutId = setTimeout(() => controller.abort(), 5000)
    
    const response = await fetch('/api/v1/orders/ocr', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json'
      },
      body: JSON.stringify(requestData),
      signal: controller.signal
    })
    
    clearTimeout(timeoutId)
    
    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`)
    }
    
    const result = await response.json()
    console.log('API 응답:', result)
    
    // 응답 데이터 저장
    if (result.data) {
      receiptData.value = {
        id: result.data.code || '',
        tel: result.data.tel || ''
      }
    }
    
    // 성공 시 결과 모달로 이동
    goToScanConfirmModal()
    
  } catch (error) {
    console.error('API 호출 실패:', error)
    // 에러가 발생해도 결과 모달로 이동 (기본값으로)
    receiptData.value = {
      id: '스캔 실패',
      tel: '스캔 실패'
    }
    goToScanConfirmModal()
  }
}
</script>

<style scoped>
.receipt-scan-container {
  min-height: 100vh;
  width: 100%;
  background: #2A2A2A;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: space-between;
  padding: 60px 0 100px 0;
  box-sizing: border-box;
}
.progress-bar {
  width: 80vw;
  max-width: 430px;
  height: 4px;
  background: linear-gradient(90deg, #7c3aed 0%, #a78bfa 100%);
  margin: 24px auto 16px auto;
  border-radius: 4px;
  box-shadow: 0 2px 8px rgba(124,58,237,0.10);
}
/* 헤더 섹션 */
.header-section {
  width: 100%;
  text-align: center;
  padding: 0 24px;
}

.scan-title-text {
  text-align: center;
  color: #fff;
  font-size: 18px;
  font-weight: 600;
  margin: 0;
  line-height: 1.4;
}
/* 카메라 섹션 */
.camera-section {
  flex: 1;
  width: 100%;
  display: flex;
  justify-content: center;
  align-items: center;
  padding: 0 24px;
}

.camera-preview {
  position: relative;
  width: 100%;
  max-width: 400px;
  height: 300px;
  border-radius: 12px;
  overflow: hidden;
}

.camera-video {
  width: 100%;
  height: 100%;
  object-fit: cover;
  border-radius: 12px;
}

.scan-frame {
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  pointer-events: none;
}

.camera-placeholder {
  width: 100%;
  height: 100%;
  background: #1F2937;
  display: flex;
  align-items: center;
  justify-content: center;
  border-radius: 12px;
}

.placeholder-content {
  text-align: center;
  color: #9CA3AF;
}

.placeholder-icon {
  font-size: 48px;
  margin-bottom: 16px;
}

.placeholder-text {
  font-size: 18px;
  font-weight: 600;
  margin: 0 0 8px 0;
  color: #E5E7EB;
}

.placeholder-subtext {
  font-size: 14px;
  margin: 0 0 20px 0;
  color: #9CA3AF;
}

.camera-permission-btn {
  background: linear-gradient(135deg, #7C3AED 0%, #6D28D9 100%);
  color: white;
  border: none;
  border-radius: 8px;
  padding: 12px 24px;
  font-size: 16px;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.2s ease;
  box-shadow: 0 2px 8px rgba(124, 60, 237, 0.3);
}

.camera-permission-btn:hover {
  transform: translateY(-1px);
  box-shadow: 0 4px 12px rgba(124, 60, 237, 0.4);
}

.camera-permission-btn:active {
  transform: translateY(0);
}
.corner {
  position: absolute;
  width: 22px;
  height: 22px;
  border: 3px solid #fff;
  border-radius: 8px;
  z-index: 2;
  box-shadow: 0 2px 8px rgba(49,46,129,0.10);
}
.tl { top: 0; left: 0; border-right: none; border-bottom: none; border-radius: 8px 0 0 0; }
.tr { top: 0; right: 0; border-left: none; border-bottom: none; border-radius: 0 8px 0 0; }
.bl { bottom: 0; left: 0; border-right: none; border-top: none; border-radius: 0 0 0 8px; }
.br { bottom: 0; right: 0; border-left: none; border-top: none; border-radius: 0 0 8px 0; }
/* 촬영 버튼 섹션 */
.capture-section {
  width: 100%;
  display: flex;
  justify-content: center;
  align-items: center;
  padding: 0 24px;
  position: relative;
  top: -30px;
  margin-top: 50px;
}

.capture-button {
  width: 64px;
  height: 64px;
  border-radius: 50%;
  background: linear-gradient(135deg, #7C3AED 0%, #6D28D9 100%);
  border: 4px solid #fff;
  box-shadow: 0 4px 16px rgba(124, 60, 237, 0.3);
  cursor: pointer;
  transition: transform 0.1s ease;
  outline: none;
  position: relative;
  display: flex;
  align-items: center;
  justify-content: center;
}

.capture-button:disabled {
  opacity: 0.7;
  cursor: not-allowed;
}

.capture-button:active:not(:disabled) {
  transform: scale(0.95);
}

.loading-spinner {
  width: 24px;
  height: 24px;
  border: 3px solid rgba(255, 255, 255, 0.3);
  border-top: 3px solid #fff;
  border-radius: 50%;
  animation: spin 1s linear infinite;
}

@keyframes spin {
  0% { transform: rotate(0deg); }
  100% { transform: rotate(360deg); }
}
/* 반응형 디자인 */
@media (max-width: 480px) {
  .receipt-scan-container {
    padding: 80px 0 100px 0;
  }
  
  .scan-title-text {
    font-size: 20px;
    margin-bottom: 20px;
  }
  
  .camera-preview {
    height: 250px;
  }
  
  .capture-button {
    width: 56px;
    height: 56px;
    border-width: 3px;
  }
}
</style>