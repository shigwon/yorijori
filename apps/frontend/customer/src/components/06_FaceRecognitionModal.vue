<template>
  <div class="modal-overlay" @click="handleOverlayClick">
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
          <button class="prev-button" @click="handlePrevious">
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
import { ref, onMounted } from 'vue'

const props = defineProps({
  capturedImage: {
    type: String,
    default: ''
  }
})

const emit = defineEmits(['previous', 'next'])

const isLoading = ref(true)

const handleOverlayClick = () => {
  // 로딩 중에는 오버레이 클릭으로 닫히지 않도록 방지
  if (isLoading.value) return
  emit('previous')
}

const handlePrevious = () => {
  emit('previous')
}

const handleNext = () => {
  emit('next')
}

onMounted(() => {
  console.log('FaceRecognitionModal 마운트됨')
  console.log('받은 이미지:', props.capturedImage ? '있음' : '없음')
  
  // 5초간 로딩 표시 후 완료 상태로 변경
  setTimeout(() => {
    console.log('로딩 완료, 완료 상태로 변경')
    console.log('완료 상태에서 이미지:', props.capturedImage ? '있음' : '없음')
    isLoading.value = false
  }, 5000)
})
</script>

<style scoped>
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
  gap: 24px;
}

.loading-spinner {
  width: 48px;
  height: 48px;
  border: 4px solid #E5E7EB;
  border-top: 4px solid #7C3AED;
  border-radius: 50%;
  animation: spin 1s linear infinite;
}

@keyframes spin {
  0% { transform: rotate(0deg); }
  100% { transform: rotate(360deg); }
}

.loading-text {
  font-size: 18px;
  font-weight: 600;
  color: #1F2937;
  margin: 0;
}

/* Completion State */
.completion-state {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 24px;
}

.captured-image-container {
  width: 120px;
  height: 120px;
  display: flex;
  align-items: center;
  justify-content: center;
  border-radius: 50%;
  overflow: hidden;
  border: 3px solid #7C3AED;
  box-shadow: 0 4px 12px rgba(124, 60, 237, 0.2);
}

.captured-image {
  width: 100%;
  height: 100%;
  object-fit: cover;
}

.dog-emoji {
  font-size: 80px;
  filter: drop-shadow(0 4px 8px rgba(0, 0, 0, 0.1));
}

.completion-text {
  font-size: 18px;
  font-weight: 600;
  color: #1F2937;
  margin: 0;
  line-height: 1.4;
}

/* Button Container */
.button-container {
  display: flex;
  gap: 12px;
  width: 100%;
  margin-top: 8px;
}

.prev-button {
  flex: 1;
  height: 48px;
  background: white;
  border: 2px solid #D1D5DB;
  border-radius: 12px;
  color: #374151;
  font-size: 16px;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.3s ease;
}

.prev-button:hover {
  background: #F9FAFB;
  border-color: #9CA3AF;
}

.prev-button:active {
  transform: translateY(1px);
}

.next-button {
  flex: 1;
  height: 48px;
  background: #7C3AED;
  border: 2px solid #7C3AED;
  border-radius: 12px;
  color: white;
  font-size: 16px;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.3s ease;
}

.next-button:hover {
  background: #6D28D9;
  border-color: #6D28D9;
  transform: translateY(-1px);
}

.next-button:active {
  transform: translateY(0);
}

/* Responsive Design */
@media (max-width: 480px) {
  .modal-content {
    padding: 32px 24px;
    max-width: 280px;
  }
  
  .loading-text,
  .completion-text {
    font-size: 16px;
  }
  
  .dog-emoji {
    font-size: 70px;
  }
  
  .button-container {
    gap: 8px;
  }
  
  .prev-button,
  .next-button {
    height: 44px;
    font-size: 15px;
  }
}
</style> 