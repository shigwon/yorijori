<template>
  <div class="manual-input-container">
    <!-- 메인 콘텐츠 -->
    <div class="main-content">
      <h1 class="title">주문정보를 입력해주세요</h1>
      
      <div class="input-container">
        <div class="input-group">
          <div class="input-header">
            <label class="input-label">주문번호</label>
            <div class="check-icon">✓</div>
          </div>
          <input 
            type="text" 
            v-model="orderNumber"
            @input="validateOrderNumber"
            class="input-field" 
            placeholder="123456"
            maxlength="6"
          />
          <div v-if="orderNumberError" class="error-message">
            주문번호를 6자리로 입력해주세요
          </div>
        </div>
        
        <div class="input-group">
          <div class="input-header">
            <label class="input-label">안심번호</label>
            <div class="eye-icon">👁</div>
          </div>
          <input 
            type="text" 
            v-model="safeNumber"
            @input="formatPhoneNumber"
            class="input-field" 
            placeholder="010-1234-5678"
            maxlength="13"
          />
          <div v-if="phoneError" class="error-message">
            전화번호를 11자리로 입력해주세요
          </div>
        </div>
      </div>
    </div>

    <!-- 액션 버튼 -->
    <div class="action-section">
      <button class="next-button" @click="handleNext" :disabled="!isFormValid">
        다음
      </button>
    </div>
  </div>

  <!-- 수동 입력 확인 모달 -->
  <ManualConfirmModal 
    v-if="showConfirmModal" 
    :order-number="orderNumber"
    :safe-number="safeNumber"
    @close="showConfirmModal = false" 
  />
</template>

<script setup>
import { ref, computed, onMounted } from 'vue'
import { useAppState } from '../composables/useAppState'
import ManualConfirmModal from './05_ManualConfirmScreen.vue'

const { setProgressPercent } = useAppState()

const orderNumber = ref('')
const safeNumber = ref('')
const showConfirmModal = ref(false)
const phoneError = ref(false)
const orderNumberError = ref(false)

onMounted(() => {
  setProgressPercent(50) // Set progress to 50% on this screen
})

const isFormValid = computed(() => {
  // 주문번호가 6자리이고, 전화번호가 11자리여야 함
  const orderValidChars = orderNumber.value.replace(/[^0-9a-zA-Z]/g, '')
  const phoneNumbersOnly = safeNumber.value.replace(/[^0-9]/g, '')
  return orderValidChars.length === 6 && 
         phoneNumbersOnly.length === 11
})

// 주문번호 유효성 검사 함수
const validateOrderNumber = () => {
  // 숫자와 영문자만 허용 (소문자, 대문자)
  const validChars = orderNumber.value.replace(/[^0-9a-zA-Z]/g, '')
  
  // 6자리로 제한
  if (validChars.length > 6) {
    orderNumber.value = validChars.slice(0, 6)
  } else {
    orderNumber.value = validChars
  }
  
  // 오류 메시지 업데이트
  orderNumberError.value = validChars.length > 0 && validChars.length !== 6
}

// 전화번호 형식 자동 변환 함수
const formatPhoneNumber = () => {
  // 숫자만 추출
  let numbers = safeNumber.value.replace(/[^0-9]/g, '')
  
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
  
  safeNumber.value = formatted
  
  // 오류 메시지 업데이트
  phoneError.value = numbers.length > 0 && numbers.length !== 11
}

const handleNext = () => {
  if (isFormValid.value) {
    // 모달 표시
    showConfirmModal.value = true
  }
}
</script>

<style scoped>
.manual-input-container {
  width: 100%;
  height: 100%;
  background: white;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: flex-start;
  padding: 20px 24px 24px 24px;
  box-sizing: border-box;
  position: relative;
}

/* 메인 콘텐츠 */
.main-content {
  flex: 1;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: flex-start;
  width: 100%;
  max-width: 400px;
  margin-top: 40px;
}

.title {
  font-size: 24px;
  font-weight: 700;
  color: #1F2937;
  text-align: center;
  margin: 0 0 40px 0;
  line-height: 1.3;
}

.input-container {
  width: 100%;
  background: white;
  border: 1px solid #E5E7EB;
  border-radius: 16px;
  padding: 24px;
  margin-bottom: 40px;
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.05);
}

.input-group {
  margin-bottom: 20px;
}

.input-group:last-child {
  margin-bottom: 0;
}

.input-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 8px;
}

.input-label {
  font-size: 14px;
  color: #6B7280;
  font-weight: 500;
}

.check-icon, .eye-icon {
  font-size: 14px;
  color: #6B7280;
}

.input-field {
  width: 100%;
  height: 48px;
  padding: 0 16px;
  border: 1px solid #E5E7EB;
  border-radius: 12px;
  font-size: 16px;
  color: #1F2937;
  background: white;
  transition: all 0.2s ease;
  box-sizing: border-box;
}

.input-field:focus {
  outline: none;
  border-color: #7C3AED;
  box-shadow: 0 0 0 3px rgba(124, 60, 237, 0.1);
}

.input-field::placeholder {
  color: #D1D5DB;
  opacity: 0.7;
}

/* 오류 메시지 스타일 */
.error-message {
  color: #DC2626;
  font-size: 12px;
  margin-top: 4px;
  font-weight: 500;
}

/* 액션 섹션 */
.action-section {
  width: 100%;
  max-width: 400px;
  padding: 0;
  margin-top: auto;
  margin-bottom: 40px;
}

.next-button {
  width: 100%;
  height: 56px;
  background: #7C3AED;
  border: none;
  border-radius: 16px;
  color: white;
  font-size: 18px;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.2s ease;
  box-shadow: 0 4px 12px rgba(124, 60, 237, 0.2);
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

/* 모바일 반응형 */
@media (max-width: 480px) {
  .manual-input-container {
    padding: 20px 16px 100px 16px;
    height: 100vh;
    box-sizing: border-box;
  }
  
  .main-content {
    margin-top: 30px;
  }
  
  .title {
    font-size: 22px;
    margin-bottom: 100px;
  }
  
  .input-container {
    padding: 20px;
    margin-bottom: 32px;
  }
  
  .input-field {
    height: 44px;
    font-size: 15px;
    
  }
  
  .next-button {
    height: 52px;
    font-size: 17px;
  }
  
  .action-section {
    margin-bottom: 20px;
  }
}
</style> 