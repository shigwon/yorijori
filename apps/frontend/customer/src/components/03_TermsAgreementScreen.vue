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
        <span class="term-text">위치정보 동의(필수)</span>
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
          <p>[제 1 장 총칙]
제 1조 목적
본 약관은 본 홈페이지를 이용하여 고객서비스 개선, 선발 및 운영을 주관하는 회사 (RiNKY)가 운영하는 서비스와 관련하여 회사와 이용자와의 권리/의무 및 책임사항을 규정함을 목적으로 합니다.

제 2 조 용어의 정의
1. "서비스"란 접속 가능한 유/무선 정보통신기기의 종류와는 상관없이 이용 가능한 "회사"가 제공하는 모든 서비스를 의미합니다.
2. 본 약관에서 사용하는 용어의 정의는 본 조에서 정하는 것을 제외하고는 관계법령 및 서비스 별 안내에서 정하는 바에 따릅니다.
제 3 조 이용약관의 효력 및 변경
1. 회사는 본 약관의 내용을 이용자가 쉽게 알 수 있도록 각 "홈페이지" 서비스의 초기 화면에 게시합니다.
2. 회사는 약관의 규제에 관한 법률, 전자거래기본법, 전자 서명법, 정보통신망이용 촉진 및 정보보호 등에 관한 법률 등 관련법을 위배하지 않는 범위에서 본 약관을 개정할 수 있습니다.
3. 회사는 본 약관을 개정할 경우에는 적용일자 및 개정사유를 명시하여 현행 약관과 함께 회사가 제공하는 "홈페이지"의 초기 화면에 그 적용일자 7일 이전부터 적용일자 전일까지 공지합니다.
다만, 회원에게 불리하게 약관내용을 변경하는 경우에는 최소한 30일 이상의 사전 유예기간을 두고 공지합니다. 이 경우 회사는 개정 전 내용과 개정 후 내용을 명확하게 비교하여 회원이 알기 쉽도록 표시합니다.
4. 회원은 개정된 약관에 대해 거부할 권리가 있습니다. 회원은 개정된 약관에 동의하지 않을 경우 서비스 이용을 중단하고 회원등록을 해지할 수 있습니다. 단, 개정된 약관의 효력 발생일 이후에도 서비스를 계속 이용할 경우에는 약관의 변경사항에 동의한 것으로 간주합니다.
5. 변경된 약관에 대한 정보를 알지 못해 발생하는 회원의 피해는 회사가 책임지지 않습니다.
제 4 조 약관의 해석
1. 본 약관은 회사가 제공하는 각 서비스 별 운영규칙과 함께 적용됩니다.
2. 본 약관에 명시되지 아니한 사항과 본 약관의 해석에 관하여는 관계법령 또는 상관례에 따릅니다.
[제 2 장 이용계약 체결]
제 5 조 이용계약의 성립
1. 이용계약은 회원의 본 이용약관 내용에 대한 동의와 이용신청에 대하여 회사의 이용승낙으로 성립합니다.
2. 본 이용약관에 대한 동의는 회원등록 당시 본 약관을 읽고 "위 서비스 약관에 동의합니다"의 대화창에 표시를 한 후 등록하기 단추를 누름으로써 의사표시를 한 것으로 간주합니다.
제 6 조 서비스 이용신청
1. 회원으로 등록하여 본 서비스를 이용하고자 하는 이용고객은 회사에서 요청하는 제반정보(주문번호, 안심심번호 등)를 제공하여야 합니다.
2. 회원은 반드시 회사가 제공하는 서비스가 요구하는 회원 본인인증 절차를 거쳐야만 서비스를 이용할 수 있으며, 본인인증을 거치지 않은 회원은 일체의 권리를 주장할 수 없습니다.
제 7 조 개인정보의 보호 및 사용
1. 회사는 관계법령이 정하는 바에 따라 회원등록정보를 포함한 회원의 개인정보를 보호하기 위해 노력합니다. 회원 개인정보의 보호 및 사용에 대해서는 관련법령 및 회사의 개인정보보호정책이 적용됩니다. 단, 회사의 공식사이트 이외의 웹에서 링크된 사이트에서는 회사의 개인정보보호정책이 적용되지 않습니다. 또한 회사는 회원의 귀책사유로 인해 노출된 정보에 대해서 일체의 책임을 지지 않습니다.
2. 회사는 이용자에게 제공하는 서비스의 양적, 질적 향상을 위하여 이용자의 개인정보를 제휴사에게 제공, 공유할 수 있으며, 이 때에는 반드시 이용자의 동의를 받아 필요한 최소한의 정보를 제공, 공유하며 누구에게 어떤 목적으로 어떤 정보를 제공, 공유하는지 명시합니다.
3. 회원은 원하는 경우 언제든 회사에 제공한 개인정보의 수집과 이용에 대한 동의를 철회할 수 있으며, 동의의 철회는 회원 탈퇴를 하는 것으로 이루어집니다.
제 8 조 이용신청의 승낙과 제한
1. 회사는 제 6조의 규정에 의한 이용신청고객에 대하여 업무 수행상 또는 기술상 지장이 없는 경우에 원칙적으로 접수순서에 따라 서비스 이용을 승낙합니다.
2. 회사는 다음 각 호의 내용에 해당하는 경우 이용신청에 대한 승낙을 제한할 수 있고, 그 사유가 해소될 때까지 승낙을 유보할 수 있습니다.
⑴ 회사의 서비스 관련설비에 여유가 없는 경우
⑵ 회사의 기술상 지장이 있는 경우
⑶ 기타 회사의 사정상 필요하다고 인정되는 경우
3. 회사는 다음 각 호의 내용에 해당하는 경우 이용신청에 대한 승낙을 하지 아니할 수도 있습니다.
⑴ 타인의 ID를 이용하여 신청한 경우
⑵ 이용계약 신청서의 내용을 허위로 기재한 경우
⑶ 사회의 안녕과 질서, 미풍양속을 저해할 목적으로 신청한 경우
⑷ 부정한 용도로 본 서비스를 이용하고자 하는 경우
⑸ 영리를 추구할 목적으로 본 서비스를 이용하고자 하는 경우
⑹ 기타 회사가 정한 등록신청 요건이 미비된 경우
⑺ 본 서비스와 경쟁관계가 있는 이용자가 신청하는 경우
⑻ 기타 규정한 제반사항을 위반하며 신청하는 경우
4. 회사는 이용신청고객이 관계법령에서 규정하는 미성년자일 경우에 서비스 별 안내에서 정하는 바에 따라 승낙을 보류할 수 있습니다.
[부칙] 본 약관은 2025년 08월 18일부터 적용됩니다.</p>
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