<template>
  <div class="terms-container">
    <!-- 환영 텍스트 -->
    <div class="welcome-text">
      <h1>아래 동의버튼을 눌러주세요</h1>
      <img src="../assets/accept.png" alt="Accept" class="accept-image" />
    </div>

    <!-- 약관 목록 -->
    <div class="terms-list">
      <!-- 전체 동의 -->
      <div class="term-item master-term">
        <div class="checkbox-container">
          <input 
            type="checkbox" 
            id="master-agreement" 
            v-model="masterAgreement"
            @change="handleMasterAgreement"
            class="checkbox"
          />
          <label for="master-agreement" class="checkbox-label"></label>
        </div>
        <span class="term-text">약관 전체동의</span>
      </div>

      <!-- 필수 약관 -->
      <div class="term-item">
        <div class="checkbox-container">
          <input 
            type="checkbox" 
            id="terms-of-use" 
            v-model="agreements.termsOfUse"
            @change="checkMasterAgreement"
            class="checkbox"
          />
          <label for="terms-of-use" class="checkbox-label"></label>
        </div>
        <span class="term-text clickable" @click="showTermsModal">이용약관 동의(필수)</span>
        <span class="arrow-icon clickable" @click="showTermsModal">›</span>
      </div>

      <div class="term-item">
        <div class="checkbox-container">
          <input 
            type="checkbox" 
            id="location-agreement" 
            v-model="agreements.locationInfo"
            @change="checkMasterAgreement"
            class="checkbox"
          />
          <label for="location-agreement" class="checkbox-label"></label>
        </div>
        <span class="term-text">위치정보동의(필수)</span>
      </div>

      <!-- 선택 약관 -->
      <div class="term-item">
        <div class="checkbox-container">
          <input 
            type="checkbox" 
            id="image-agreement" 
            v-model="agreements.imageUse"
            @change="checkMasterAgreement"
            class="checkbox"
          />
          <label for="image-agreement" class="checkbox-label"></label>
        </div>
        <div class="term-content">
          <span class="term-text">이미지 사용 동의(필수)</span>
          <p class="term-description">배달 완료 후 이미지는 삭제가 됩니다.</p>
        </div>
      </div>
    </div>

    <!-- 다음 버튼 -->
    <div class="button-container">
      <button 
        class="next-button" 
        :class="{ 'disabled': !isButtonEnabled }"
        @click="handleNext"
        :disabled="!isButtonEnabled"
      >
        다음
      </button>
    </div>

    <!-- 약관 모달 -->
    <div v-if="showModal" class="modal-overlay" @click="closeModal">
      <div class="modal-content" @click.stop>
        <div class="modal-header">
          <h3>이용약관</h3>
          <button class="close-button" @click="closeModal">×</button>
        </div>
        <div class="modal-body">
          <p>응 너네 정보 팔아 넘길겨 ㅋㅋ</p>
        </div>
        <div class="modal-footer">
          <button class="modal-button" @click="closeModal">확인</button>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, computed } from 'vue'
import { useAppState } from '../composables/useAppState'

const { goToPhotoSelection } = useAppState()

const masterAgreement = ref(false)
const agreements = ref({
  termsOfUse: false,
  locationInfo: false,
  imageUse: false
})
const showModal = ref(false)

const isButtonEnabled = computed(() => {
  return agreements.value.termsOfUse && agreements.value.locationInfo
})

const handleMasterAgreement = () => {
  if (masterAgreement.value) {
    agreements.value.termsOfUse = true
    agreements.value.locationInfo = true
    agreements.value.imageUse = true
  } else {
    agreements.value.termsOfUse = false
    agreements.value.locationInfo = false
    agreements.value.imageUse = false
  }
}

const checkMasterAgreement = () => {
  masterAgreement.value = agreements.value.termsOfUse && 
                          agreements.value.locationInfo && 
                          agreements.value.imageUse
}

const handleNext = () => {
  if (isButtonEnabled.value) {
    console.log('약관 동의 완료')
    goToPhotoSelection()
  }
}

const showTermsModal = () => {
  showModal.value = true
}

const closeModal = () => {
  showModal.value = false
}
</script>

<style scoped>
.terms-container {
  width: 100%;
  height: 100%;
  background: white;
  display: flex;
  flex-direction: column;
  padding: 80px 24px 20px 24px;
  position: relative;
}

.accept-image {
  width: 150px !important;
  height: 150px !important;
  object-fit: contain;
  margin-top: 20px;
  min-width: 150px !important;
  min-height: 150px !important;
  max-width: none !important;
  max-height: none !important;
}

/* Welcome Text */
.welcome-text {
  text-align: center;
  margin-bottom: 50px;
  margin-top: 30px;
}

.welcome-text h1 {
  font-size: 24px;
  font-weight: 700;
  color: #1F2937;
  margin: 0;
}

/* Terms List */
.terms-list {
  flex: 1;
  display: flex;
  flex-direction: column;
  gap: 20px;
  margin-top: -50px;
}

.term-item {
  display: flex;
  align-items: flex-start;
  gap: 12px;
  padding: 16px 0;
  border-bottom: 1px solid #F3F4F6;
}

.term-item:last-child {
  border-bottom: none;
}

.master-term {
  border-bottom: 2px solid #E5E7EB;
  padding-bottom: 20px;
  margin-bottom: 10px;
}

.master-term .term-text {
  font-weight: 600;
  font-size: 16px;
}


.checkbox-container {
  flex-shrink: 0;
  position: relative;
}

.checkbox {
  position: absolute;
  opacity: 0;
  cursor: pointer;
  height: 0;
  width: 0;
}

.checkbox-label {
  display: block;
  width: 20px;
  height: 20px;
  border: 2px solid #D1D5DB;
  border-radius: 50%;
  cursor: pointer;
  position: relative;
  transition: all 0.2s ease;
}

.checkbox:checked + .checkbox-label {
  background: #7C3AED;
  border-color: #7C3AED;
}

.checkbox:checked + .checkbox-label::after {
  content: '';
  position: absolute;
  left: 6px;
  top: 2px;
  width: 6px;
  height: 10px;
  border: solid white;
  border-width: 0 2px 2px 0;
  transform: rotate(45deg);
}

/* Term Text */
.term-text {
  flex: 1;
  font-size: 15px;
  color: #374151;
  line-height: 1.4;
}

.term-content {
  flex: 1;
  display: flex;
  flex-direction: column;
  gap: 4px;
}

.term-description {
  font-size: 13px;
  color: #9CA3AF;
  margin: 0;
  line-height: 1.3;
}

/* Arrow Icon */
.arrow-icon {
  color: #9CA3AF;
  font-size: 18px;
  font-weight: 300;
}

/* Clickable Elements */
.clickable {
  cursor: pointer;
  transition: color 0.2s ease;
}

.clickable:hover {
  color: #7C3AED;
}

/* Button Container */
.button-container {
  margin-top: 0;
  margin-bottom: 80px;
  padding: 20px 24px;
  position: fixed;
  top: 540px;
  left: 0;
  right: 0;
  z-index: 1000;
}

.next-button {
  width: 100%;
  height: 56px;
  background: #E5E7EB;
  border: none;
  border-radius: 16px;
  color: #9CA3AF;
  font-size: 18px;
  font-weight: 600;
  cursor: not-allowed;
  transition: all 0.3s ease;
  box-shadow: none;
}

.next-button:not(.disabled) {
  background: linear-gradient(135deg, #7C3AED 0%, #6D28D9 100%);
  color: white;
  cursor: pointer;
  box-shadow: 0 4px 20px rgba(124, 60, 237, 0.2);
}

.next-button:not(.disabled):hover {
  background: #6D28D9;
  transform: translateY(-2px);
  box-shadow: 0 6px 25px rgba(124, 60, 237, 0.3);
}

.next-button:not(.disabled):active {
  transform: translateY(0);
}

/* Modal Styles */
.modal-overlay {
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background: rgba(0, 0, 0, 0.5);
  display: flex;
  align-items: center;
  justify-content: center;
  z-index: 2000;
}

.modal-content {
  background: white;
  border-radius: 16px;
  width: 90%;
  max-width: 400px;
  max-height: 80vh;
  overflow: hidden;
  box-shadow: 0 10px 30px rgba(0, 0, 0, 0.3);
}

.modal-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 20px 24px;
  border-bottom: 1px solid #E5E7EB;
}

.modal-header h3 {
  margin: 0;
  font-size: 18px;
  font-weight: 600;
  color: #1F2937;
}

.close-button {
  background: none;
  border: none;
  font-size: 24px;
  color: #9CA3AF;
  cursor: pointer;
  padding: 0;
  width: 24px;
  height: 24px;
  display: flex;
  align-items: center;
  justify-content: center;
}

.close-button:hover {
  color: #6B7280;
}

.modal-body {
  padding: 24px;
  max-height: 60vh;
  overflow-y: auto;
}

.modal-body p {
  margin: 0;
  font-size: 16px;
  line-height: 1.5;
  color: #374151;
}

.modal-footer {
  padding: 20px 24px;
  border-top: 1px solid #E5E7EB;
  display: flex;
  justify-content: flex-end;
}

.modal-button {
  background: #7C3AED;
  color: white;
  border: none;
  border-radius: 8px;
  padding: 12px 24px;
  font-size: 16px;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.3s ease;
}

.modal-button:hover {
  background: #6D28D9;
}

/* Responsive Design */
@media (max-width: 480px) {
  .terms-container {
    padding: 16px 20px;
  }
  
  .welcome-text h1 {
    font-size: 20px;
  }
  
  .term-text {
    font-size: 14px;
  }
  
  .term-description {
    font-size: 12px;
  }
  
  .accept-image {
    width: 80px;
    height: 80px;
  }
}
</style> 