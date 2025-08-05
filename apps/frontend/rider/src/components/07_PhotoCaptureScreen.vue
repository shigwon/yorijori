<template>
  <div class="photo-capture-container">
    <!-- Camera View Section (상단 2/3) -->
    <div class="camera-section">
      <div class="camera-preview">
        <!-- 실제 카메라 비디오 -->
        <video ref="videoRef" autoplay playsinline class="camera-video" v-if="!cameraError"></video>
        
        <!-- 카메라 플레이스홀더 -->
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

        <!-- Light Overlay -->
        <div class="light-overlay" v-if="!cameraError">
          <div class="scan-area"></div>
        </div>

        <!-- Scan Frame -->
        <div class="scan-frame" v-if="!cameraError">
          <div class="corner corner-top-left"></div>
          <div class="corner corner-top-right"></div>
          <div class="corner corner-bottom-left"></div>
          <div class="corner corner-bottom-right"></div>
        </div>

        <!-- Instruction Text Overlay -->
        <div class="instruction-text" v-if="!cameraError">
          사진을 촬영해주세요
        </div>
      </div>
    </div>

    <!-- White Information Panel (하단 1/3) -->
    <div class="info-panel">
      <h2 class="panel-title">사진 촬영</h2>
      
      <div class="instructions">
        <p class="instruction-line">반드시 음식을 담아둔 로봇만 사진을 찍어주세요</p>
        <p class="instruction-line">빈 로봇을 촬영하면 주문 확인이 불가능합니다.</p>
      </div>

      <button class="capture-button" @click="handleCapture" :disabled="isCapturing">
        {{ isCapturing ? '촬영 중...' : '촬영' }}
      </button>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted, onUnmounted } from 'vue'
import { useAppState } from '../composables/useAppState'

const { goToComplete, orderData } = useAppState()

const videoRef = ref(null)
const isCameraReady = ref(false)
const isCapturing = ref(false)
const cameraError = ref(false)
const cameraStatus = ref('카메라 로딩 중...')
let stream = null

// 카메라 시작
const startCamera = async () => {
  console.log('카메라 시작 시도...')
  console.log('브라우저:', navigator.userAgent)
  console.log('HTTPS 여부:', window.location.protocol === 'https:')
  
  cameraStatus.value = '카메라 초기화 중...'
  
  try {
    // 기존 스트림이 있다면 먼저 정리
    if (stream) {
      stopCamera()
    }
    
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
    stream = await navigator.mediaDevices.getUserMedia(constraints)
    console.log('카메라 스트림 성공:', stream)
    
    if (videoRef.value) {
      videoRef.value.srcObject = stream
      
      // 비디오 로드 이벤트
      videoRef.value.onloadedmetadata = () => {
        console.log('비디오 메타데이터 로드됨')
        cameraStatus.value = '카메라 준비 완료'
        isCameraReady.value = true
        cameraError.value = false
      }
      
      videoRef.value.oncanplay = () => {
        console.log('비디오 재생 가능')
        cameraStatus.value = '카메라 준비 완료'
        isCameraReady.value = true
        cameraError.value = false
      }
      
      videoRef.value.onplay = () => {
        console.log('비디오 재생 시작')
        cameraStatus.value = '카메라 실행 중'
        isCameraReady.value = true
        cameraError.value = false
      }
      
      videoRef.value.onerror = (e) => {
        console.log('비디오 오류:', e)
        cameraStatus.value = '카메라 오류 발생'
        cameraError.value = true
      }
    }
  } catch (error) {
    console.error('카메라 접근 실패:', error)
    console.log('에러 이름:', error.name)
    console.log('에러 메시지:', error.message)
    
    if (error.name === 'NotAllowedError') {
      cameraStatus.value = '카메라 권한이 거부되었습니다'
      console.log('카메라 권한이 거부되었습니다. 브라우저 설정에서 카메라 권한을 허용해주세요.')
    } else if (error.name === 'NotFoundError') {
      cameraStatus.value = '카메라를 찾을 수 없습니다'
      console.log('카메라를 찾을 수 없습니다.')
    } else if (error.name === 'NotReadableError') {
      cameraStatus.value = '카메라가 다른 앱에서 사용 중입니다'
      console.log('카메라가 다른 앱에서 사용 중입니다.')
    } else if (error.name === 'OverconstrainedError') {
      cameraStatus.value = '지원하지 않는 카메라 설정입니다'
      console.log('지원하지 않는 카메라 설정입니다.')
      // 더 간단한 설정으로 재시도
      try {
        const simpleStream = await navigator.mediaDevices.getUserMedia({ 
          video: { facingMode: 'environment' },
          audio: false 
        })
        if (videoRef.value) {
          videoRef.value.srcObject = simpleStream
          await videoRef.value.play()
          cameraError.value = false
          cameraStatus.value = '카메라 실행 중'
        }
      } catch (simpleErr) {
        console.log('간단한 설정도 실패:', simpleErr)
        cameraError.value = true
        cameraStatus.value = '카메라 설정 실패'
      }
    } else {
      cameraStatus.value = '카메라 접근 실패: ' + error.message
    }
    
    cameraError.value = true
    isCameraReady.value = false
  }
}

// 카메라 중지
const stopCamera = () => {
  if (stream) {
    stream.getTracks().forEach(track => track.stop())
    stream = null
  }
  isCameraReady.value = false
}

// 카메라 권한 요청
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
    
    if (videoRef.value) {
      videoRef.value.srcObject = stream
      
      // 비디오 이벤트 리스너 추가
      videoRef.value.onloadedmetadata = () => {
        console.log('비디오 메타데이터 로드됨')
        cameraStatus.value = '카메라 준비 완료'
        cameraError.value = false
        isCameraReady.value = true
      }
      
      videoRef.value.oncanplay = () => {
        console.log('비디오 재생 가능')
        cameraStatus.value = '카메라 준비 완료'
        cameraError.value = false
        isCameraReady.value = true
      }
      
      videoRef.value.onplay = () => {
        console.log('비디오 재생 시작')
        cameraStatus.value = '카메라 실행 중'
        cameraError.value = false
        isCameraReady.value = true
      }
      
      videoRef.value.onerror = (e) => {
        console.log('비디오 오류:', e)
        cameraStatus.value = '카메라 오류 발생'
        cameraError.value = true
      }
      
      // 재생 시도
      try {
        await videoRef.value.play()
        console.log('카메라 권한 허용됨 - 재생 성공')
        cameraStatus.value = '카메라 실행 중'
      } catch (playErr) {
        console.log('재생 실패:', playErr)
        // 재생 실패해도 스트림은 연결됨
        cameraStatus.value = '카메라 연결됨 (재생 대기 중)'
        cameraError.value = false
        isCameraReady.value = true
      }
    } else {
      console.log('videoRef가 없음')
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

// 카메라 다시 시도
const retryCamera = async () => {
  cameraError.value = false
  isCameraReady.value = false
  await startCamera()
}

// 이미지 다운로드 함수
const downloadImage = (url, blob) => {
  const a = document.createElement('a')
  a.href = url
  a.download = `lingki_photo_${new Date().getTime()}.jpg`
  document.body.appendChild(a)
  a.click()
  document.body.removeChild(a)
  URL.revokeObjectURL(url)
  console.log('사진 다운로드 완료')
}

// 사진 촬영
const handleCapture = async () => {
  if (isCapturing.value) return
  
  console.log('사진 촬영 시작...')
  isCapturing.value = true
  
  try {
    // Canvas 생성
    const canvas = document.createElement('canvas')
    const context = canvas.getContext('2d')
    
    // 비디오가 준비되었는지 확인
    if (!videoRef.value || !videoRef.value.videoWidth) {
      console.log('비디오가 준비되지 않음')
      goToComplete()
      return
    }
    
    // 비디오 크기에 맞춰 Canvas 설정
    canvas.width = videoRef.value.videoWidth
    canvas.height = videoRef.value.videoHeight
    
    console.log('Canvas 크기:', canvas.width, 'x', canvas.height)
    
    // 비디오 프레임을 Canvas에 그리기
    context.drawImage(videoRef.value, 0, 0, canvas.width, canvas.height)
    
    // Canvas를 base64로 변환
    const base64Image = canvas.toDataURL('image/jpeg', 0.9)
    const base64Only = base64Image.split(',')[1] // "data:image/jpeg;base64," 부분 제거
    
    console.log('사진 촬영 완료, base64 길이:', base64Only.length)
    
    // 즉시 완료 화면으로 이동 (사용자 경험 개선)
    console.log('완료 화면으로 이동...')
    goToComplete()
    
    // 백그라운드에서 사진 전송
    processPhotoAsync(base64Only)
    
  } catch (error) {
    console.error('사진 촬영 오류:', error)
    // 오류가 발생해도 완료 화면으로 이동
    goToComplete()
  } finally {
    isCapturing.value = false
  }
}

// 백그라운드에서 사진 처리
const processPhotoAsync = async (base64Image) => {
  try {
    // base64 → Blob 변환
    const blob = await (await fetch(`data:image/jpeg;base64,${base64Image}`)).blob()

    // FormData 생성
    const formData = new FormData()
    formData.append('orderCode', orderData.value.orderNumber || 'example') // 실제 주문코드 사용
    formData.append('fileCategory', 'FOOD') // 또는 FACE
    formData.append('file', blob, 'photo.jpg')

    console.log('사진 API 요청 시작...')

    // fetch 요청
    const response = await fetch('/api/v1/files', {
      method: 'POST',
      body: formData
  
    })

    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`)
    }

    const result = await response.json()
    console.log('사진 API 응답:', result)

  } catch (error) {
    console.error('사진 API 호출 실패:', error)
    // API 실패해도 이미 완료 화면으로 이동했으므로 추가 처리 불필요
  }
}

onMounted(() => {
  startCamera()
})

onUnmounted(() => {
  stopCamera()
})
</script>

<style scoped>
/* 모바일 최적화 전체 컨테이너 */
.photo-capture-container {
  height: 100vh;
  background: #000;
  display: flex;
  flex-direction: column;
  position: relative;
  width: 100%;
  box-sizing: border-box;
  /* 모바일 성능 최적화 */
  -webkit-overflow-scrolling: touch;
  overscroll-behavior: none;
}

/* Camera Section (상단 2/3) */
.camera-section {
  flex: 2;
  position: relative;
  overflow: hidden;
  width: 100%;
  box-sizing: border-box;
  /* 모바일에서 카메라 뷰 최적화 */
  -webkit-transform: translateZ(0);
  transform: translateZ(0);
}

.camera-preview {
  width: 100%;
  height: 100%;
  position: relative;
  overflow: hidden;
}

.camera-video {
  width: 100%;
  height: 100%;
  object-fit: cover;
  position: absolute;
  top: 0;
  left: 0;
}

.camera-placeholder {
  width: 100%;
  height: 100%;
  background: #1F2937;
  display: flex;
  align-items: center;
  justify-content: center;
  position: absolute;
  top: 0;
  left: 0;
}

.placeholder-content {
  text-align: center;
  color: white;
  padding: 20px;
}

.placeholder-icon {
  font-size: 48px;
  margin-bottom: 16px;
}

.placeholder-text {
  font-size: 18px;
  font-weight: 600;
  margin-bottom: 8px;
}

.placeholder-subtext {
  font-size: 14px;
  opacity: 0.8;
  margin-bottom: 20px;
}

/* Light Overlay */
.light-overlay {
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background: rgba(0, 0, 0, 0.2);
  z-index: 3;
}

.scan-area {
  position: absolute;
  top: 50%;
  left: 50%;
  transform: translate(-50%, -50%);
  width: 320px;
  height: 280px;
  background: transparent;
  border-radius: 8px;
  box-shadow: inset 0 0 0 1000px rgba(255, 255, 255, 0.05);
}

/* Scan Frame */
.scan-frame {
  position: absolute;
  top: 50%;
  left: 50%;
  transform: translate(-50%, -50%);
  width: 320px;
  height: 280px;
  z-index: 5;
}

.corner {
  position: absolute;
  width: 30px;
  height: 30px;
  border: 3px solid white;
}

.corner-top-left {
  top: 0;
  left: 0;
  border-right: none;
  border-bottom: none;
}

.corner-top-right {
  top: 0;
  right: 0;
  border-left: none;
  border-bottom: none;
}

.corner-bottom-left {
  bottom: 0;
  left: 0;
  border-right: none;
  border-top: none;
}

.corner-bottom-right {
  bottom: 0;
  right: 0;
  border-left: none;
  border-top: none;
}

/* Instruction Text */
.instruction-text {
  position: absolute;
  top: 50%;
  left: 50%;
  transform: translate(-50%, -50%);
  color: white;
  font-size: 18px;
  font-weight: 500;
  text-align: center;
  z-index: 4;
  margin-top: -160px;
  text-shadow: 0 2px 4px rgba(0, 0, 0, 0.5);
}

/* Camera Error */
.camera-error {
  position: absolute;
  top: 50%;
  left: 50%;
  transform: translate(-50%, -50%);
  color: white;
  text-align: center;
  z-index: 6;
  background: rgba(0, 0, 0, 0.9);
  padding: 24px;
  border-radius: 16px;
  max-width: 280px;
  width: 90%;
}

.camera-error p {
  margin: 0 0 12px 0;
  font-size: 16px;
  line-height: 1.4;
}

.camera-error p:last-of-type {
  margin-bottom: 16px;
  font-size: 14px;
  opacity: 0.8;
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

.retry-button {
  background: #7C3AED;
  color: white;
  border: none;
  border-radius: 12px;
  padding: 12px 24px;
  font-size: 14px;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.3s ease;
}

.retry-button:hover {
  background: #6D28D9;
  transform: translateY(-1px);
}

.retry-button:active {
  transform: translateY(0);
}

/* White Information Panel (하단 1/3) */
.info-panel {
  flex: 1;
  background: white;
  padding: 24px 16px;
  border-radius: 20px 20px 0 0;
  box-shadow: 0 -4px 20px rgba(0, 0, 0, 0.1);
  width: 100%;
  box-sizing: border-box;
  display: flex;
  flex-direction: column;
  justify-content: space-between;
}

.panel-title {
  font-size: 30px;
  font-weight: 700;
  color: #1F2937;
  margin: 0 0 16px 0;
  text-align: center;
}

.instructions {
  margin-bottom: 24px;
  flex: 1;
}

.instruction-line {
  font-size: 22px;
  color: #374151;
  margin: 0 0 8px 0;
  line-height: 1.4;
  text-align: center;
}

.instruction-line:last-child {
  margin-bottom: 0;
}

.capture-button {
  width: 100%;
  height: 56px;
  background: #7C3AED;
  border: none;
  border-radius: 16px;
  color: white;
  font-size: 16px;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.3s ease;
  box-shadow: 0 4px 20px rgba(124, 60, 237, 0.2);
  margin-top: auto;
}

.capture-button:hover:not(:disabled) {
  background: #6D28D9;
  transform: translateY(-2px);
  box-shadow: 0 6px 25px rgba(124, 60, 237, 0.3);
}

.capture-button:active:not(:disabled) {
  transform: translateY(0);
}

/* 모바일 터치 최적화 */
@media (hover: none) and (pointer: coarse) {
  .capture-button {
    -webkit-tap-highlight-color: transparent;
    touch-action: manipulation;
  }
  
  .capture-button:active:not(:disabled) {
    background: #5B21B6;
    transform: scale(0.98);
  }
}

.capture-button:disabled {
  background: #D1D5DB;
  color: #9CA3AF;
  cursor: not-allowed;
  transform: none;
  box-shadow: none;
}

/* 모바일 비율 최적화 */
@media (max-width: 480px) {
  .photo-capture-container {
    height: 100vh;
    width: 100vw;
    overflow: hidden;
  }
  
  .camera-section {
    flex: 2.5;
    width: 100%;
    box-sizing: border-box;
  }
  
  .camera-preview {
    width: 100%;
    height: 100%;
  }
  
  .info-panel {
    flex: 1;
    width: 100%;
    box-sizing: border-box;
    padding: 20px 16px;
    border-radius: 16px 16px 0 0;
  }
  
  .scan-frame {
    width: 240px;
    height: 200px;
  }
  
  .scan-area {
    width: 240px;
    height: 200px;
  }
  
  .instruction-text {
    font-size: 16px;
    margin-top: -140px;
    padding: 0 20px;
    line-height: 1.3;
  }
  
  .panel-title {
    font-size: 17px;
    margin-bottom: 12px;
  }
  
  .instructions {
    margin-bottom: 16px;
  }
  
  .instruction-line {
    font-size: 13px;
    line-height: 1.4;
    margin-bottom: 6px;
  }
  
  .capture-button {
    height: 52px;
    font-size: 16px;
    border-radius: 14px;
  }
  
  .camera-error {
    padding: 20px;
    max-width: 260px;
  }
  
  .camera-error p {
    font-size: 15px;
  }
  
  .camera-error p:last-of-type {
    font-size: 13px;
  }
  
  .retry-button {
    padding: 10px 20px;
    font-size: 13px;
  }
}

/* 작은 모바일 화면 (iPhone SE 등) */
@media (max-width: 375px) {
  .camera-section {
    flex: 2.2;
  }
  
  .info-panel {
    flex: 1;
    padding: 16px 12px;
  }
  
  .scan-frame {
    width: 220px;
    height: 180px;
  }
  
  .scan-area {
    width: 220px;
    height: 180px;
  }
  
  .instruction-text {
    font-size: 15px;
    margin-top: -130px;
  }
  
  .panel-title {
    font-size: 16px;
    margin-bottom: 10px;
  }
  
  .instruction-line {
    font-size: 12px;
    margin-bottom: 5px;
  }
  
  .capture-button {
    height: 48px;
    font-size: 15px;
  }
}

/* 세로 모드에서 안전 영역 고려 */
@supports (padding: max(0px)) {
  .photo-capture-container {
    padding-top: env(safe-area-inset-top);
    padding-bottom: env(safe-area-inset-bottom);
  }
  
  .info-panel {
    padding-bottom: max(24px, env(safe-area-inset-bottom) + 20px);
  }
}

/* 모바일에서 하단 버튼이 보이도록 여백 추가 */
@media (max-width: 480px) {
  .photo-capture-container {
    height: calc(100vh - 35px);
    padding-bottom: 35px;
  }
  
  .info-panel {
    padding-bottom: 15px;
    margin-bottom: 0;
  }
  
  .capture-button {
    margin-bottom: 8px;
    position: relative;
    z-index: 10;
  }
}

/* 작은 모바일 화면에서 추가 여백 */
@media (max-width: 375px) {
  .photo-capture-container {
    height: calc(100vh - 40px);
    padding-bottom: 40px;
  }
  
  .info-panel {
    padding-bottom: 12px;
    margin-bottom: 0;
  }
  
  .capture-button {
    margin-bottom: 6px;
    position: relative;
    z-index: 10;
  }
}

/* 매우 작은 화면에서 추가 여백 */
@media (max-width: 320px) {
  .photo-capture-container {
    height: calc(100vh - 45px);
    padding-bottom: 45px;
  }
  
  .info-panel {
    padding-bottom: 10px;
    margin-bottom: 0;
  }
  
  .capture-button {
    margin-bottom: 4px;
    position: relative;
    z-index: 10;
  }
}
</style> 