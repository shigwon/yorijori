<template>
  <div class="modal-overlay">
    <div class="center-box">
      <div class="modal-content">
        <div class="loading-icon">
          <div class="spinner"></div>
        </div>
        <h2 class="modal-title">영수증을 분석하고 있어요</h2>
        <p class="loading-text">잠시만 기다려주세요...</p>
        <div class="progress-container">
          <div class="progress-bar">
            <div class="progress-fill" :style="{ width: progressPercent + '%' }"></div>
          </div>
          <span class="progress-text">{{ Math.round(progressPercent) }}%</span>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted, onUnmounted } from 'vue'
import { useAppState } from '../composables/useAppState'

const { goToScanConfirmModal, receiptData } = useAppState()

const progressPercent = ref(0)
let progressInterval = null
let timeoutId = null

onMounted(() => {
  // 5초 동안 진행률을 0%에서 100%로 증가
  const duration = 5000 // 5초
  const interval = 50 // 50ms마다 업데이트
  const increment = (interval / duration) * 100
  
  progressInterval = setInterval(() => {
    progressPercent.value += increment
    if (progressPercent.value >= 100) {
      progressPercent.value = 100
      clearInterval(progressInterval)
      // 5초 후 타임아웃 - 기본값으로 결과 모달로 이동
      handleTimeout()
    }
  }, interval)
  
  // 5초 후 강제 타임아웃 (안전장치)
  timeoutId = setTimeout(() => {
    if (progressInterval) {
      clearInterval(progressInterval)
    }
    handleTimeout()
  }, 5000)
})

const handleTimeout = () => {
  // 타임아웃 시 기본값 설정
  if (!receiptData.value.id || receiptData.value.id === '') {
    receiptData.value = {
      id: '스캔 중...',
      tel: '스캔 중...'
    }
  }
  goToScanConfirmModal()
}

onUnmounted(() => {
  if (progressInterval) {
    clearInterval(progressInterval)
  }
  if (timeoutId) {
    clearTimeout(timeoutId)
  }
})
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
  justify-content: center;
  min-height: 100vh;
  padding: 0 24px;
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
  text-align: center;
}

.loading-icon {
  margin-bottom: 24px;
}

.spinner {
  width: 48px;
  height: 48px;
  border: 4px solid #f3f4f6;
  border-top: 4px solid #7c3aed;
  border-radius: 50%;
  animation: spin 1s linear infinite;
}

@keyframes spin {
  0% { transform: rotate(0deg); }
  100% { transform: rotate(360deg); }
}

.modal-title {
  font-size: 20px;
  font-weight: 600;
  color: #1f2937;
  margin: 0 0 12px 0;
  line-height: 1.4;
}

.loading-text {
  font-size: 16px;
  color: #6b7280;
  margin: 0 0 32px 0;
  line-height: 1.5;
}

.progress-container {
  width: 100%;
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 12px;
}

.progress-bar {
  width: 100%;
  height: 8px;
  background: #f3f4f6;
  border-radius: 4px;
  overflow: hidden;
}

.progress-fill {
  height: 100%;
  background: linear-gradient(90deg, #7c3aed 0%, #a78bfa 100%);
  border-radius: 4px;
  transition: width 0.1s ease;
}

.progress-text {
  font-size: 14px;
  font-weight: 600;
  color: #7c3aed;
}

@media (max-width: 480px) {
  .center-box {
    padding: 0 16px;
  }
  
  .modal-content {
    padding: 32px 20px 24px 20px;
  }
  
  .modal-title {
    font-size: 18px;
  }
  
  .loading-text {
    font-size: 14px;
  }
}
</style> 