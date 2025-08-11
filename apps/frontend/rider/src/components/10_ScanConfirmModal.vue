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
                @input="validateOrderNumber"
                type="text" 
                class="info-input"
                placeholder="주문번호를 입력하세요"
                maxlength="6"
              />
              <span class="edit-icon">✏️</span>
            </div>
          </div>
          <div v-if="orderNumberError" class="error-message">
            주문번호를 6자리로 입력해주세요
          </div>
          <div class="info-item">
            <span class="info-label">안심번호 :</span>
            <div class="input-container">
              <input 
                v-model="editedReceiptData.tel" 
                @input="formatPhoneNumber"
                type="text" 
                class="info-input"
                placeholder="010-1234-5678"
                maxlength="13"
              />
              <span class="edit-icon">✏️</span>
            </div>
          </div>
          <div v-if="phoneError" class="error-message">
            전화번호를 11자리로 입력해주세요
          </div>
        </div>
        <button class="next-button" @click="handleNext" :disabled="!isFormValid">다음</button>
        <p class="help-text" @click="handleProblemClick">주문정보에 문제가 생기셨나요?</p>
      </div>
      <div class="bottom-text">주문정보가 스캔되었어요.</div>
    </div>
  </div>
</template>

<script setup>
import { useAppState } from '../composables/useAppState'
import { ref, onMounted, computed } from 'vue'
import { useRouter } from 'vue-router'

const { goToScanOption, receiptData } = useAppState()
const router = useRouter()

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

// 오류 상태
const orderNumberError = ref(false)
const phoneError = ref(false)

onMounted(() => {
  // 스캔된 데이터로 초기화
  editedReceiptData.value = {
    id: receiptData.value.id || '',
    tel: receiptData.value.tel || ''
  }
  
  // 초기 유효성 검사
  validateOrderNumber()
  formatPhoneNumber()
})

// 폼 유효성 검사
const isFormValid = computed(() => {
  const orderValidChars = editedReceiptData.value.id.replace(/[^0-9a-zA-Z]/g, '')
  const phoneNumbersOnly = editedReceiptData.value.tel.replace(/[^0-9]/g, '')
  return orderValidChars.length === 6 && phoneNumbersOnly.length === 11
})

// 주문번호 유효성 검사 함수
const validateOrderNumber = () => {
  // 숫자와 영문자만 허용 (소문자, 대문자)
  const validChars = editedReceiptData.value.id.replace(/[^0-9a-zA-Z]/g, '')
  
  // 6자리로 제한
  if (validChars.length > 6) {
    editedReceiptData.value.id = validChars.slice(0, 6)
  } else {
    editedReceiptData.value.id = validChars
  }
  
  // 오류 메시지 업데이트
  orderNumberError.value = validChars.length > 0 && validChars.length !== 6
}

// 전화번호 형식 자동 변환 함수
const formatPhoneNumber = () => {
  // 숫자만 추출
  let numbers = editedReceiptData.value.tel.replace(/[^0-9]/g, '')
  
  // 11자리 이하로 제한
  if (numbers.length > 11) {
    numbers = numbers.slice(0, 11)
  }
  
  // 형식에 맞게 하이픈 추가
  let formatted = ''
  if (numbers.length <= 3) {
    formatted = numbers
  } else if (numbers.length <= 7) {
    formatted = numbers.slice(0, 3) + '-' + numbers.slice(3)
  } else {
    formatted = numbers.slice(0, 3) + '-' + numbers.slice(3, 7) + '-' + numbers.slice(7)
  }
  
  editedReceiptData.value.tel = formatted
  
  // 오류 메시지 업데이트
  phoneError.value = numbers.length > 0 && numbers.length !== 11
}

const handleNext = () => {
  console.log('다음 버튼 클릭됨 - handleNext 시작')
  
  // 수정된 데이터를 receiptData에 저장
  receiptData.value = {
    id: editedReceiptData.value.id,
    tel: editedReceiptData.value.tel
  }
  
  console.log('receiptData 저장 완료:', receiptData.value)
  
  // localStorage에 저장하여 페이지 이동 시에도 데이터 유지
  localStorage.setItem('receiptData', JSON.stringify(receiptData.value))
  console.log('localStorage에 receiptData 저장 완료')
  
  // API 호출은 하되 기다리지 않음 (백그라운드에서 실행)
  fetch('/api/v1/orders/create', {
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
  .then(response => {
    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`)
    }
    return response.json()
  })
  .then(result => {
    console.log('주문정보 API 응답:', result)
  })
  .catch(error => {
    console.error('주문정보 API 호출 실패:', error)
  })
  
  console.log('모달 닫기 시작')
  // 모달 닫기
  window.closeScanConfirmModal()
  
  // 로딩 모달도 강제로 닫기
  if (window.closeLoadingModal) {
    window.closeLoadingModal()
  }
  
  console.log('라우터 네비게이션 시작')
  // 바로 LocationRequest 화면으로 이동 (강제 이동)
  window.location.href = '/rider/location-request'
  
  console.log('handleNext 완료')
}

const handleProblemClick = () => {
  console.log('주문정보 문제 클릭됨 - handleProblemClick 시작')
  
  // 모달 닫기
  window.closeScanConfirmModal()
  
  // 로딩 모달도 강제로 닫기
  if (window.closeLoadingModal) {
    window.closeLoadingModal()
  }
  
  // 스캔 옵션 화면으로 강제 이동
  window.location.href = '/rider/scan-option'
  console.log('스캔 옵션 화면으로 강제 이동')
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
  padding-top: 170px;
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
  width: 80%;
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
  width: 65%;
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

/* 오류 메시지 스타일 */
.error-message {
  color: #DC2626;
  font-size: 12px;
  margin-top: 4px;
  font-weight: 500;
  text-align: right;
  width: 100%;
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

.next-button:hover:not(:disabled) {
  background: #6D28D9;
  transform: translateY(-1px);
  box-shadow: 0 6px 16px rgba(124, 60, 237, 0.3);
}

.next-button:active:not(:disabled) {
  transform: translateY(0);
}

.next-button:disabled {
  background: #D1D5DB;
  color: #9CA3AF;
  cursor: not-allowed;
  box-shadow: none;
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