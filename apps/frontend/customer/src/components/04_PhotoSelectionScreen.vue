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
</template>

<script setup>
import { ref } from 'vue'
import { useAppState } from '../composables/useAppState'

const { goToCameraCapture, openFaceRecognitionModal } = useAppState()

const fileInput = ref(null)

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
      
      // 부모 컴포넌트에 선택된 이미지 전달
      emit('photo-selected', {
        file: file,
        url: `data:${file.type};base64,${base64Image}`,
        base64: base64Image
      })
      
      // 얼굴 인식 모달 표시 (약간의 지연 후)
      setTimeout(() => {
        console.log('앨범 선택 후 얼굴 인식 모달 표시 이벤트 발생')
        openFaceRecognitionModal(base64Image)
      }, 100)
      
    } catch (error) {
      console.error('파일 처리 오류:', error)
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
      emit('photo-uploaded', result)
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
</style> 