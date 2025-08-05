<template>
  <div class="modal-overlay">
    <div class="center-box">
      <div class="modal-content">
        <h2 class="modal-title">스캔한 주문정보를 확인해주세요</h2>
        <p class="edit-hint">💡 수정이 필요한 경우 직접 입력하세요</p>
        <div class="info-box">
          <div class="info-item">
            <span class="info-label">주문번호 :</span>
            <div class="input-container">
              <input 
                v-model="editedReceiptData.id" 
                type="text" 
                class="info-input"
                placeholder="주문번호를 입력하세요"
              />
              <span class="edit-icon">✏️</span>
            </div>
          </div>
          <div class="info-item">
            <span class="info-label">안심번호 :</span>
            <div class="input-container">
              <input 
                v-model="editedReceiptData.tel" 
                type="text" 
                class="info-input"
                placeholder="안심번호를 입력하세요"
              />
              <span class="edit-icon">✏️</span>
            </div>
          </div>
        </div>
        <button class="next-button" @click="handleNext">다음</button>
        <p class="help-text" @click="handleProblemClick">주문정보에 문제가 생기셨나요?</p>
      </div>
      <div class="bottom-text">주문정보가 스캔되었어요.</div>
    </div>
  </div>
</template>

<script setup>
import { useAppState } from '../composables/useAppState'
import { ref, onMounted } from 'vue'

const { goToLocationRequest, goToScanOption, receiptData } = useAppState()

// URL에서 robotId 파라미터 가져오기
const getRobotId = () => {
  const urlParams = new URLSearchParams(window.location.search)
  const robotId = urlParams.get('robotId')
  return robotId ? parseInt(robotId) : 1 
}

// 수정 가능한 데이터
const editedReceiptData = ref({
  id: '',
  tel: ''
})

onMounted(() => {
  // 스캔된 데이터로 초기화
  editedReceiptData.value = {
    id: receiptData.value.id || '',
    tel: receiptData.value.tel || ''
  }
})

const handleNext = async () => {
  // 수정된 데이터를 receiptData에 저장
  receiptData.value = {
    id: editedReceiptData.value.id,
    tel: editedReceiptData.value.tel
  }
  
  try {
    console.log('주문정보 API 요청 시작...')
    
  
    const response = await fetch('/api/v1/orders/create', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({
        robotId: getRobotId(),
        code: editedReceiptData.value.id,
        tel: editedReceiptData.value.tel
      })
    })
    
    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`)
    }
    
    const result = await response.json()
    console.log('주문정보 API 응답:', result)
    
    // API 성공 시 위치정보 동의 화면으로 이동
    goToLocationRequest(editedReceiptData.value.id, editedReceiptData.value.tel)
    
  } catch (error) {
    console.error('주문정보 API 호출 실패:', error)
    // API 실패 시에도 위치정보 동의 화면으로 이동 (사용자 경험 개선)
    goToLocationRequest(editedReceiptData.value.id, editedReceiptData.value.tel)
  }
}

const handleProblemClick = () => {
  goToScanOption()
}
</script>

<style scoped>
.modal-overlay {
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background: rgba(0, 0, 0, 0.8);
  display: flex;
  align-items: center;
  justify-content: center;
  z-index: 1000;
  backdrop-filter: blur(8px);
}
.center-box {
  width: 100%;
  max-width: 400px;
  margin: 0 auto;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: flex-start; /* 위로 정렬 */
  min-height: 100vh;
  padding-top: 100px;
}
.modal-content {
  width: 100%;
  background: #fff;
  border-radius: 24px;
  box-shadow: 0 8px 32px rgba(49,46,129,0.18);
  display: flex;
  flex-direction: column;
  align-items: center;
  padding: 40px 24px 30px 24px;
  margin-bottom: 16px;
  min-height: 400px;
}
.modal-title {
  font-size: 20px;
  font-weight: 700;
  color: #1F2937;
  text-align: center;
  margin-bottom: 30px;
  line-height: 1.4;
}
.info-box {
  width: 100%;
  background: #F3F4F6;
  border-radius: 12px;
  padding: 32px 24px;
  margin-bottom: 30px;
  box-shadow: 0 2px 8px rgba(0,0,0,0.05);
}
.info-item {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 12px;
}
.info-item:last-child { margin-bottom: 0; }
.info-label {
  font-size: 16px;
  color: #1F2937;
  font-weight: 500;
}
.info-value {
  font-size: 16px;
  color: #1F2937;
  font-weight: 600;
}

.info-input {
  font-size: 16px;
  color: #1F2937;
  font-weight: 600;
  background: transparent;
  border: none;
  outline: none;
  text-align: right;
  width: 60%;
  padding: 4px 8px;
  border-radius: 4px;
  transition: background-color 0.2s ease;
}

.info-input:focus {
  background-color: rgba(124, 58, 237, 0.1);
}

.info-input::placeholder {
  color: #9CA3AF;
  font-weight: 400;
}

.edit-hint {
  font-size: 14px;
  color: #7C3AED;
  text-align: center;
  margin: 0 0 20px 0;
  font-weight: 500;
}

.input-container {
  position: relative;
  display: flex;
  align-items: center;
  width: 60%;
}

.edit-icon {
  position: absolute;
  right: 8px;
  font-size: 14px;
  opacity: 0.6;
  pointer-events: none;
}

.info-input:focus + .edit-icon {
  opacity: 1;
  color: #7C3AED;
}
.next-button {
  width: 100%;
  height: 56px;
  background: #7C3AED;
  border: none;
  border-radius: 12px;
  color: white;
  font-size: 18px;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.2s ease;
  box-shadow: 0 4px 12px rgba(124, 60, 237, 0.2);
  margin-bottom: 24px;
}
.next-button:hover {
  background: #6D28D9;
  transform: translateY(-1px);
  box-shadow: 0 6px 16px rgba(124, 60, 237, 0.3);
}
.next-button:active {
  transform: translateY(0);
}
.help-text {
  font-size: 14px;
  color: #9CA3AF;
  text-align: center;
  margin: 0;
  cursor: pointer;
  transition: color 0.2s ease;
}
.help-text:hover {
  color: #7C3AED;
}
.bottom-text {
  text-align: center;
  color: #FFFFFF;
  font-size: 15px;
  margin-top: 8px;
  margin-bottom: 0;
}
@media (max-width: 480px) {
  .center-box {
    max-width: 95vw;
  }
  .modal-content {
    padding: 30px 16px 20px 16px;
    min-height: 350px;
  }
  .modal-title {
    font-size: 18px;
    margin-bottom: 25px;
  }
  .info-box {
    padding: 28px 20px;
    margin-bottom: 25px;
  }
  .info-label, .info-value {
    font-size: 15px;
  }
  .next-button {
    height: 48px;
    font-size: 16px;
    margin-bottom: 20px;
  }
  .bottom-text {
    font-size: 13px;
    margin-top: 6px;
  }
}
</style>