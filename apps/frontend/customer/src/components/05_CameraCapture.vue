<template>
  <div class="camera-capture-container">
    <!-- 카메라 뷰 섹션 (상단 2/3) -->
    <div class="camera-section">
      <div class="camera-preview">
        <!-- 실제 카메라 비디오 -->
        <video ref="videoElement" autoplay playsinline class="camera-video" v-if="!cameraError"></video>
        
        <!-- 카메라 플레이스홀더 -->
        <div class="camera-placeholder" v-if="cameraError">
          <div class="placeholder-content">
            <div class="placeholder-icon">📷</div>
            <p class="placeholder-text">{{ cameraStatus }}</p>
            <p class="placeholder-subtext">카메라 권한을 허용해주세요</p>
            <button class="camera-permission-btn" @click="startCamera">
              카메라 권한 허용
            </button>
          </div>
        </div>

        <!-- 조명 오버레이 -->
        <div class="light-overlay" v-if="!cameraError">
          <div class="scan-area"></div>
        </div>

        <!-- 스캔 프레임 -->
        <div class="scan-frame" v-if="!cameraError">
          <div class="corner corner-top-left"></div>
          <div class="corner corner-top-right"></div>
          <div class="corner corner-bottom-left"></div>
          <div class="corner corner-bottom-right"></div>
        </div>

        <!-- 안내 텍스트 오버레이 -->
        <div class="instruction-text" v-if="!cameraError">
          얼굴을 촬영해주세요
        </div>
      </div>
    </div>

    <!-- 흰색 정보 패널 (하단 1/3) -->
    <div class="info-panel">
      <h2 class="panel-title">사진 촬영</h2>
      
      <div class="instructions">
        <p class="instruction-line">로봇 도착 시 본인확인에 사용됩니다</p>
        <p class="instruction-line">사진은 배달이 도착즉시 삭제됩니다</p>
      </div>

      <!-- 카메라 컨트롤 -->
      <div class="camera-controls">
        <div class="spacer"></div>
        <button class="capture-button" @click="captureImage" :disabled="isCapturing">
          <div class="capture-button-inner"></div>
        </button>
        <img src="/src/assets/selfcamera.png" alt="Self Camera" class="selfcamera-icon" @click="toggleCameraMode" />
      </div>
    </div>

    <canvas ref="canvas" style="display: none;"></canvas>
  </div>
</template>

<script setup>
import { ref, onMounted, onUnmounted } from 'vue'

const emit = defineEmits(['image-captured', 'image-uploaded', 'show-face-recognition'])

const videoElement = ref(null)
const canvas = ref(null)
const cameraError = ref(false)
const cameraStatus = ref('카메라 로딩 중...')
const isCapturing = ref(false)
const isFrontCamera = ref(true) // true: 전면 카메라, false: 후면 카메라
let stream = null

const startCamera = async () => {
  console.log('카메라 시작...')
  cameraError.value = false
  cameraStatus.value = '카메라 초기화 중...'
  
  try {
    // 기존 스트림 정리
    if (stream) {
      stream.getTracks().forEach(track => track.stop())
      stream = null
    }
    
    // 모바일에서 더 안정적인 카메라 설정
    const constraints = {
      video: {
        facingMode: isFrontCamera.value ? 'user' : 'environment',
        width: { ideal: 1280 },
        height: { ideal: 720 }
      },
      audio: false
    }
    
    console.log('카메라 제약 조건:', constraints)
    stream = await navigator.mediaDevices.getUserMedia(constraints)
    console.log('카메라 스트림 성공:', stream)
    
    if (videoElement.value) {
      videoElement.value.srcObject = stream
      
      videoElement.value.onloadedmetadata = () => {
        console.log('비디오 메타데이터 로드됨')
        console.log('비디오 크기:', videoElement.value.videoWidth, 'x', videoElement.value.videoHeight)
        cameraStatus.value = '카메라 준비 완료'
      }
      
      videoElement.value.oncanplay = () => {
        console.log('비디오 재생 가능')
        cameraStatus.value = '카메라 준비 완료'
      }
      
      videoElement.value.onerror = (e) => {
        console.log('비디오 오류:', e)
        cameraError.value = true
        cameraStatus.value = '카메라 오류 발생'
      }
      
      // 모바일에서 추가 이벤트 리스너
      videoElement.value.onplay = () => {
        console.log('비디오 재생 시작')
      }
    }
  } catch (error) {
    console.error('카메라 접근 실패:', error)
    cameraError.value = true
    
    if (error.name === 'NotAllowedError') {
      cameraStatus.value = '카메라 권한이 거부되었습니다'
    } else if (error.name === 'NotFoundError') {
      cameraStatus.value = '카메라를 찾을 수 없습니다'
    } else if (error.name === 'NotReadableError') {
      cameraStatus.value = '카메라가 다른 앱에서 사용 중입니다'
    } else {
      cameraStatus.value = '카메라 접근 실패: ' + error.message
    }
  }
}

const toggleCameraMode = async () => {
  isFrontCamera.value = !isFrontCamera.value
  await startCamera()
}

const captureImage = async () => {
  if (isCapturing.value) return
  
  console.log('사진 촬영 시작...')
  isCapturing.value = true
  
  try {
    // 모바일에서 비디오 요소가 준비되었는지 확인
    if (!videoElement.value) {
      console.error('비디오 요소가 없음')
      return
    }
    
    // 비디오가 로드되었는지 확인
    if (videoElement.value.readyState < 2) {
      console.log('비디오가 아직 로드되지 않음, 잠시 대기...')
      await new Promise(resolve => setTimeout(resolve, 1000))
    }
    
    if (!videoElement.value.videoWidth || !videoElement.value.videoHeight) {
      console.error('비디오 크기가 설정되지 않음')
      return
    }
    
    console.log('비디오 크기:', videoElement.value.videoWidth, 'x', videoElement.value.videoHeight)
    
    const canvas = document.createElement('canvas')
    const context = canvas.getContext('2d')
    
    // 모바일에서 적절한 크기로 설정
    canvas.width = videoElement.value.videoWidth
    canvas.height = videoElement.value.videoHeight
    
    // 이미지 그리기
    context.drawImage(videoElement.value, 0, 0, canvas.width, canvas.height)
    
    // 모바일에서 품질 조정
    const base64Image = canvas.toDataURL('image/jpeg', 0.8)
    const base64Only = base64Image.split(',')[1]
    
    console.log('사진 촬영 완료, 크기:', base64Only.length)
    
    // 백엔드로 이미지 전송 (에러가 있어도 계속 진행)
    try {
      await sendImageToBackend(base64Only)
    } catch (uploadError) {
      console.warn('백엔드 전송 실패, 계속 진행:', uploadError)
    }
    
    // 부모 컴포넌트에 전달
    emit('image-captured', base64Only)
    
    // 얼굴 인식 모달 표시 (약간의 지연 후)
    setTimeout(() => {
      console.log('얼굴 인식 모달 표시 이벤트 발생')
      console.log('전달할 이미지 크기:', base64Only.length)
      emit('show-face-recognition', base64Only)
    }, 100)
    
  } catch (error) {
    console.error('사진 촬영 오류:', error)
  } finally {
    isCapturing.value = false
  }
}

const sendImageToBackend = async (base64Image) => {
  try {
    console.log('백엔드로 이미지 전송 중...')
    
    // base64 → Blob 변환
    const blob = await (await fetch(`data:image/jpeg;base64,${base64Image}`)).blob()
    
    // FormData 생성
    const formData = new FormData()
    formData.append('orderCode', 'example') // 실제 주문코드로 변경 필요
    formData.append('fileCategory', 'FACE') // 얼굴 사진
    formData.append('file', blob, 'face.jpg')
    
    console.log('FormData 생성 완료:', {
      orderCode: 'example',
      fileCategory: 'FACE',
      fileName: 'face.jpg',
      fileSize: blob.size
    })
    
    // fetch 요청
    const response = await fetch('/api/v1/files', {
      method: 'POST',
      body: formData
    })
    
    if (response.ok) {
      const result = await response.json()
      console.log('이미지 전송 성공:', result)
      emit('image-uploaded', result)
    } else {
      console.error('이미지 전송 실패:', response.status)
      throw new Error(`HTTP error! status: ${response.status}`)
    }
    
  } catch (error) {
    console.error('백엔드 전송 오류:', error)
    throw error
  }
}

onMounted(() => {
  startCamera()
})

onUnmounted(() => {
  if (stream) {
    stream.getTracks().forEach(track => track.stop())
  }
})
</script>

<style scoped>
/* 모바일 최적화 전체 컨테이너 */
.camera-capture-container {
  height: 100vh;
  background: #000;
  display: flex;
  flex-direction: column;
  position: relative;
  width: 100%;
  box-sizing: border-box;
  -webkit-overflow-scrolling: touch;
  overscroll-behavior: none;
}

/* Camera Section (상단 2/3) */
.camera-section {
  flex: 2.5;
  position: relative;
  overflow: hidden;
  width: 100%;
  box-sizing: border-box;
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

/* White Information Panel (하단 1/3) */
.info-panel {
  flex: 1.5;
  background: white;
  padding: 20px 16px;
  border-radius: 20px 20px 0 0;
  box-shadow: 0 -4px 20px rgba(0, 0, 0, 0.1);
  width: 100%;
  box-sizing: border-box;
  display: flex;
  flex-direction: column;
  min-height: 250px;
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
  flex: 0 0 auto;
  max-height: 80px;
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

/* Camera Controls */
.camera-controls {
  display: flex;
  align-items: center;
  justify-content: space-between;
  margin-top: auto;
  padding: 20px 0;
  flex: 0 0 auto;
  min-height: 80px;
  margin: 20px 0 0 0;
  position: relative;
}

.spacer {
  width: 64px;
}

.selfcamera-icon {
  width: 40px;
  height: 40px;
  object-fit: contain;
  cursor: pointer;
  transition: all 0.3s ease;
  margin-right: 16px;
}

.selfcamera-icon:hover {
  transform: scale(1.1);
}

.camera-mode-btn:hover {
  background: #E5E7EB;
  transform: translateY(-1px);
}

.camera-mode-btn:active {
  transform: translateY(0);
}

.capture-button {
  width: 48px;
  height: 48px;
  border-radius: 50%;
  background: white;
  border: none;
  box-shadow: 0 0 0 6px #000000;
  cursor: pointer;
  transition: all 0.3s ease;
  display: flex;
  align-items: center;
  justify-content: center;
  position: relative;
}

.capture-button-inner {
  width: 32px;
  height: 32px;
  border-radius: 50%;
  background: #000000;
  border: 3px solid white;
}

.capture-button:hover:not(:disabled) {
  transform: scale(1.05);
  box-shadow: 0 0 0 8px #000000;
}

.capture-button:active:not(:disabled) {
  transform: scale(0.95);
}

.capture-button:disabled {
  opacity: 0.5;
  cursor: not-allowed;
  transform: none;
}

.capture-button:disabled .capture-button-inner {
  background: #D1D5DB;
}

/* 모바일 비율 최적화 */
@media (max-width: 480px) {
  .camera-capture-container {
    height: 100vh;
    width: 100vw;
    overflow: hidden;
  }
  
  .camera-section {
    flex: 2.2;
    width: 100%;
    box-sizing: border-box;
  }
  
  .camera-preview {
    width: 100%;
    height: 100%;
  }
  
  .info-panel {
    flex: 1.3;
    width: 100%;
    box-sizing: border-box;
    padding: 18px 16px;
    border-radius: 16px 16px 0 0;
    min-height: 220px;
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
  
  .camera-controls {
    padding: 16px 0;
    min-height: 70px;
    margin: 16px 0 0 0;
  }

  .spacer {
    width: 56px;
  }
  
  .camera-mode-btn {
    width: 48px;
    height: 48px;
    font-size: 20px;
  }
  
  .capture-button {
    width: 44px;
    height: 44px;
  }

  .capture-button-inner {
    width: 28px;
    height: 28px;
  }

  .selfcamera-icon {
    width: 36px;
    height: 36px;
    margin-right: 12px;
  }
}

/* 작은 모바일 화면 (iPhone SE 등) */
@media (max-width: 375px) {
  .camera-section {
    flex: 2;
  }
  
  .info-panel {
    flex: 1.2;
    padding: 14px 12px;
    min-height: 180px;
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
    width: 40px;
    height: 40px;
  }

  .capture-button-inner {
    width: 24px;
    height: 24px;
  }

  .selfcamera-icon {
    width: 32px;
    height: 32px;
    margin-right: 8px;
  }

  .spacer {
    width: 48px;
  }

  .camera-controls {
    padding: 12px 0;
    min-height: 60px;
    margin: 12px 0 0 0;
  }
}
</style> 