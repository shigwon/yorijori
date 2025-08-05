<template>
  <div class="modal-overlay">
    <div class="center-box">
      <div class="modal-content">
        <h2 class="modal-title">입력한 주문정보를 확인해주세요</h2>
        <div class="info-box">
          <div class="info-item">
            <span class="info-label">주문번호 :</span>
            <span class="info-value">{{ orderNumber }}</span>
          </div>
          <div class="info-item">
            <span class="info-label">안심번호 :</span>
            <span class="info-value">{{ safeNumber }}</span>
          </div>
        </div>
        <button class="next-button" @click="handleNext">다음</button>
        <p class="help-text" @click="handleHelp">주문정보에 문제가 생기셨나요?</p>
      </div>
      <div class="bottom-text">주문정보가 입력되었어요.</div>
    </div>
  </div>
</template>

<script setup>
import { useAppState } from '../composables/useAppState'
import { onMounted } from 'vue'

const { goToLocationRequest, goToScanOption, setProgressPercent } = useAppState()

// Props 정의
const props = defineProps({
  orderNumber: {
    type: String,
    default: ''
  },
  safeNumber: {
    type: String,
    default: ''
  }
})

// Emits 정의
const emit = defineEmits(['close'])

onMounted(() => {
  setProgressPercent(60) // Set progress to 60% on this screen
})

const handleNext = () => {
  goToLocationRequest()
}

const handleHelp = () => {
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
  padding-top: 180px;
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
