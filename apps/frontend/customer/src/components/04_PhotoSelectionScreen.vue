<template>
  <div class="photo-selection-container">
    <!-- 앨범 선택용 숨겨진 파일 입력 -->
    <input 
      type="file" 
      ref="fileInput" 
      accept="image/*" 
      @change="handleFileSelect" 
      style="display: none;"
    />
    
    <!-- 채팅 메시지들 -->
    <div class="chat-container">
              <!-- 링키 메시지 1 -->
      <div class="message lingki-message">
        <div class="avatar">
          <img src="../assets/robot.png" alt="robot" class="robot-icon" />
        </div>
        <div class="message-content">
          <div class="message-bubble">
            <p>링키를 이용해주셔서 감사합니다.</p>
          </div>
          <div class="message-time">09:25 AM</div>
        </div>
      </div>

              <!-- 사용자 메시지 1 -->
      <div class="message user-message">
        <div class="message-content">
          <div class="message-bubble">
            <p>나는 셀카가 싫어</p>
          </div>
          <div class="message-time">09:25 AM</div>
        </div>
      </div>

              <!-- 링키 메시지 2 -->
      <div class="message lingki-message">
        <div class="avatar">
          <img src="../assets/robot.png" alt="robot" class="robot-icon" />
        </div>
        <div class="message-content">
          <div class="message-bubble">
            <p>저희는 앨범에서 사진을 선택하는게 가능합니다.</p>
          </div>
          <div class="message-time">09:25 AM</div>
        </div>
      </div>

              <!-- 사용자 메시지 2 -->
      <div class="message user-message">
        <div class="message-content">
          <div class="message-bubble">
            <p>제 사진이 해킹당하면 어떻게 해요</p>
          </div>
          <div class="message-time">09:25 AM</div>
        </div>
      </div>

              <!-- 링키 메시지 3 -->
      <div class="message lingki-message">
        <div class="avatar">
          <img src="../assets/robot.png" alt="robot" class="robot-icon" />
        </div>
        <div class="message-content">
          <div class="message-bubble">
            <p>저희는 고객님의 정보를 음식 수령 후 바로 삭제되니 걱정안하셔두 됩니다</p>
          </div>
          <div class="message-time">09:25 AM</div>
        </div>
      </div>
    </div>

    <!-- 액션 버튼들 -->
    <div class="action-buttons">
             <button class="action-button album-button" @click="selectFromAlbum">
         <div class="button-icon">
           <img src="../assets/album.png" alt="album" class="album-icon" />
         </div>
         <span>앨범에서 선택하기</span>
       </button>
      <button class="action-button camera-button" @click="takeSelfie">
        <div class="button-icon">
          <img src="../assets/camera.png" alt="camera" class="camera-icon" />
        </div>
        <span>셀카 찍기</span>
      </button>
    </div>
  </div>

  <!-- 얼굴 인식 모달 -->
  <div v-if="showFaceRecognitionModal" class="modal-overlay" @click="closeFaceRecognitionModal">
    <div class="modal-content" @click.stop>
      <!-- 로딩 상태 -->
      <div v-if="isLoading" class="loading-state">
        <div class="loading-spinner"></div>
        <h2 class="loading-text">인식중입니다</h2>
      </div>

      <!-- 완료 상태 -->
      <div v-else class="completion-state">
        <div class="captured-image-container">
          <img v-if="capturedImage" :src="capturedImage" alt="촬영된 사진" class="captured-image" />
          <div v-else class="dog-emoji">🐕</div>
        </div>
        <h2 class="completion-text">얼굴 인식이 완료되었어요!</h2>
        
        <div class="button-container">
          <button class="prev-button" @click="closeFaceRecognitionModal">
            이전
          </button>
          <button class="next-button" @click="handleNext">
            다음
          </button>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref } from 'vue'
import { useAppState } from '../composables/useAppState'

const { goToCameraCapture, goToLocationSetting, capturedImage } = useAppState()

const fileInput = ref(null)
const showFaceRecognitionModal = ref(false)
const isLoading = ref(true)

const selectFromAlbum = () => {
  console.log('앨범에서 선택하기 클릭됨')
  // 실제 파일 선택 다이얼로그 열기
  fileInput.value.click()
}

const handleFileSelect = async (event) => {
  const file = event.target.files[0]
  if (file) {
    console.log('선택된 파일:', file.name)
    
    try {
      // 파일을 Base64로 변환
      const base64Image = await fileToBase64(file)
      console.log('이미지 Base64 변환 완료')
      
      // 백엔드로 이미지 전송 (에러가 있어도 계속 진행)
      try {
        await sendImageToBackend(base64Image, file.name)
      } catch (uploadError) {
        console.warn('백엔드 전송 실패, 계속 진행:', uploadError)
      }
      
      // 얼굴 인식 모달 표시 (즉시)
      console.log('앨범 선택 후 얼굴 인식 모달 표시')
      
      // useAppState에 선택한 사진 저장
      capturedImage.value = `data:image/jpeg;base64,${base64Image}`
      console.log('useAppState에 앨범 사진 저장 완료')
      
      // 직접 모달 표시
      showFaceRecognitionModal.value = true
      isLoading.value = true
      
      console.log('얼굴 인식 모달 표시 완료')
      
      // 5초 후 로딩 완료
      setTimeout(() => {
        console.log('로딩 완료, 완료 상태로 변경')
        isLoading.value = false
      }, 5000)
      
    } catch (error) {
      console.error('파일 처리 오류:', error)
      // 오류 발생 시에도 모달 표시 시도
      setTimeout(() => {
        console.log('파일 처리 오류로 인한 얼굴 인식 모달 표시')
        if (window.openFaceRecognitionModal) {
          window.openFaceRecognitionModal('')
        } else {
          openFaceRecognitionModal('')
        }
      }, 100)
    }
  }
}

const fileToBase64 = (file) => {
  return new Promise((resolve, reject) => {
    const reader = new FileReader()
    reader.onload = () => {
      const base64 = reader.result.split(',')[1]
      resolve(base64)
    }
    reader.onerror = reject
    reader.readAsDataURL(file)
  })
}

const sendImageToBackend = async (base64Image, fileName) => {
  try {
    console.log('백엔드로 앨범 이미지 전송 중...')
    
    // base64 → Blob 변환
    const blob = await (await fetch(`data:image/jpeg;base64,${base64Image}`)).blob()
    
    // FormData 생성
    const formData = new FormData()
    formData.append('orderCode', 'example') // 실제 주문코드로 변경 필요
    formData.append('fileCategory', 'FACE') // 얼굴 사진
    formData.append('file', blob, fileName)
    
    const response = await fetch('/api/v1/files', {
      method: 'POST',
      body: formData
    })
    
    if (response.ok) {
      const result = await response.json()
      console.log('앨범 이미지 전송 성공:', result)
    } else {
      console.error('앨범 이미지 전송 실패:', response.status)
      // 에러를 throw하지 않고 로그만 출력
    }
    
  } catch (error) {
    console.error('백엔드 전송 오류:', error)
    // 에러를 throw하지 않고 로그만 출력
  }
}

const takeSelfie = () => {
  console.log('셀카 찍기 클릭됨')
  goToCameraCapture()
}

const closeFaceRecognitionModal = () => {
  showFaceRecognitionModal.value = false
  console.log('얼굴 인식 모달 닫기')
}

const handleNext = () => {
  closeFaceRecognitionModal()
  console.log('다음 버튼 클릭 - 위치 설정 화면으로 이동')
  goToLocationSetting()
}
</script>

<style scoped>
.photo-selection-container {
  width: 100%;
  height: 100%;
  background: white;
  display: flex;
  flex-direction: column;
  padding: 20px 24px 40px 24px;
  position: relative;
  overflow-y: auto;
}

/* Chat Container */
.chat-container {
  flex: 1;
  display: flex;
  flex-direction: column;
  gap: 8px;
  margin-bottom: 40px;
  min-height: 0;
  margin-top: 40px;
}

/* Message Styles */
.message {
  display: flex;
  gap: 8px;
  max-width: 80%;
}

.lingki-message {
  align-self: flex-start;
}

.user-message {
  align-self: flex-end;
  flex-direction: row-reverse;
}

/* Avatar */
.avatar {
  flex-shrink: 0;
}

.robot-icon {
  width: 32px;
  height: 32px;
  object-fit: contain;
  border-radius: 50%;
}

/* Message Content */
.message-content {
  display: flex;
  flex-direction: column;
  gap: 4px;
}

.message-bubble {
  padding: 12px 16px;
  border-radius: 18px;
  max-width: 100%;
}

.lingki-message .message-bubble {
  background: #F3F4F6;
  color: #1F2937;
}

.user-message .message-bubble {
  background: #7C3AED;
  color: white;
}

.message-bubble p {
  margin: 0;
  font-size: 14px;
  line-height: 1.4;
}

.message-time {
  font-size: 11px;
  color: #9CA3AF;
  margin-left: 4px;
}

.user-message .message-time {
  text-align: right;
  margin-right: 4px;
}

/* Action Buttons */
.action-buttons {
  display: flex;
  gap: 12px;
  margin-bottom: 45px;
  flex-shrink: 0;
  border: 1px solid #E5E7EB;
  border-radius: 12px;
  padding: 16px;
  background: #FAFAFA;
  margin-left: -12px;
  margin-right: -12px;
  margin-top: -150px;
  position: fixed;
  top: 650px;
  left: 0;
  right: 0;
  z-index: 1000;
}

.action-button {
  flex: 1;
  height: 100px;
  border: 2px solid #7C3AED;
  border-radius: 12px;
  background: white;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  gap: 6px;
  cursor: pointer;
  transition: all 0.3s ease;
  padding: 12px;
}

.action-button:hover {
  background: #F3F4F6;
  transform: translateY(-2px);
}

.action-button:active {
  transform: translateY(0);
}

.button-icon {
  font-size: 32px;
  margin-bottom: 4px;
  display: flex;
  align-items: center;
  justify-content: center;
}

.camera-icon {
  width: 32px;
  height: 32px;
  object-fit: contain;
}

.album-icon {
  width: 32px;
  height: 32px;
  object-fit: contain;
}

.action-button span {
  font-size: 14px;
  font-weight: 600;
  color: #7C3AED;
  text-align: center;
}

/* Responsive Design */
@media (max-width: 480px) {
  .photo-selection-container {
    padding: 16px 20px 60px 20px;
  }
  
  .message {
    max-width: 85%;
  }
  
  .action-button {
    height: 90px;
  }
  
  .button-icon {
    font-size: 24px;
  }
  
  .album-icon {
    width: 28px;
    height: 28px;
  }
  
  .album-icon::before {
    width: 18px;
    height: 14px;
    top: 5px;
    left: 5px;
  }
  
  .album-icon::after {
    width: 5px;
    height: 5px;
    top: 7px;
    left: 7px;
  }
  
  .action-button span {
    font-size: 12px;
  }
}

/* 얼굴 인식 모달 스타일 */
.modal-overlay {
  position: fixed;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  background: rgba(0, 0, 0, 0.8);
  display: flex;
  align-items: center;
  justify-content: center;
  z-index: 1000;
}

.modal-content {
  background: white;
  border-radius: 20px;
  padding: 40px 32px;
  max-width: 320px;
  width: 90%;
  text-align: center;
  box-shadow: 0 20px 40px rgba(0, 0, 0, 0.3);
}

/* 로딩 상태 */
.loading-state {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 20px;
}

.loading-spinner {
  width: 60px;
  height: 60px;
  border: 4px solid #f3f3f3;
  border-top: 4px solid #7C3AED;
  border-radius: 50%;
  animation: spin 1s linear infinite;
}

@keyframes spin {
  0% { transform: rotate(0deg); }
  100% { transform: rotate(360deg); }
}

.loading-text {
  font-size: 24px;
  font-weight: 600;
  color: #1F2937;
  margin: 0;
}

/* 완료 상태 */
.completion-state {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 20px;
}

.captured-image-container {
  width: 120px;
  height: 120px;
  border-radius: 50%;
  overflow: hidden;
  background: #f3f4f6;
  display: flex;
  align-items: center;
  justify-content: center;
  margin-bottom: 10px;
}

.captured-image {
  width: 100%;
  height: 100%;
  object-fit: cover;
}

.dog-emoji {
  font-size: 60px;
}

.completion-text {
  font-size: 20px;
  font-weight: 600;
  color: #1F2937;
  margin: 0;
  line-height: 1.4;
}

.button-container {
  display: flex;
  gap: 12px;
  margin-top: 10px;
}

.prev-button, .next-button {
  padding: 12px 24px;
  border: none;
  border-radius: 8px;
  font-size: 16px;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.2s ease;
}

.prev-button {
  background: #f3f4f6;
  color: #374151;
}

.prev-button:hover {
  background: #e5e7eb;
}

.next-button {
  background: linear-gradient(135deg, #7C3AED 0%, #6D28D9 100%);
  color: white;
}

.next-button:hover {
  transform: translateY(-1px);
  box-shadow: 0 4px 12px rgba(124, 60, 237, 0.4);
}
</style> 