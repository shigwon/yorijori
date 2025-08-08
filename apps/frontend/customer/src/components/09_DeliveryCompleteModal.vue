<template>
  <div class="modal-overlay" @click="closeModal">
    <div class="modal-content" @click.stop>
      <!-- Title -->
      <div class="title">음식이 도착했어요!</div>
      
      <!-- Robot Scan Image -->
      <div class="face-image-container">
        <img 
          src="../assets/robotscan.png" 
          alt="로봇 스캔" 
          class="face-image"
        />
      </div>
      
      <!-- Instruction Text -->
      <div class="instruction">얼굴을 인식해서 음식을 받아주세요</div>
    </div>
  </div>
</template>

<script setup>
import { onMounted } from 'vue'
import { useAppState } from '../composables/useAppState'

const { goToFoodCompartment } = useAppState()

const closeModal = () => {
  // 모달 닫기 로직 (필요시 구현)
}

onMounted(() => {
  console.log('배달 완료 모달 마운트됨')
  
  // 4초 후 음식 칸 화면으로 전환
  setTimeout(() => {
    console.log('4초 후 자동으로 음식 칸 화면으로 전환')
    goToFoodCompartment()
  }, 4000)
})
</script>

<style scoped>
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
  z-index: 1000;
}

.modal-content {
  background: white;
  border-radius: 20px;
  padding: 40px 30px;
  text-align: center;
  max-width: 320px;
  width: 90%;
  box-shadow: 0 10px 30px rgba(0, 0, 0, 0.2);
}

.title {
  font-size: 20px;
  font-weight: 700;
  color: #1F2937;
  margin-bottom: 30px;
}

.face-image-container {
  display: flex;
  justify-content: center;
  margin-bottom: 30px;
}

.face-image {
  width: 150px;
  height: 150px;
  border-radius: 50%;
  object-fit: cover;
  border: 4px solid #7C3AED;
  box-shadow: 0 4px 12px rgba(124, 58, 237, 0.3);
}

.face-placeholder {
  width: 120px;
  height: 120px;
  border-radius: 50%;
  background: #E5E7EB;
  display: flex;
  align-items: center;
  justify-content: center;
  border: 4px solid #7C3AED;
  box-shadow: 0 4px 12px rgba(124, 58, 237, 0.3);
}

.face-placeholder span {
  font-size: 48px;
  color: #9CA3AF;
}

.instruction {
  font-size: 16px;
  color: #6B7280;
  line-height: 1.4;
}

/* Responsive Design */
@media (max-width: 480px) {
  .modal-content {
    padding: 30px 20px;
    max-width: 280px;
  }
  
  .title {
    font-size: 18px;
    margin-bottom: 25px;
  }
  
  .face-image,
  .face-placeholder {
    width: 130px;
    height: 130px;
  }
  
  .face-placeholder span {
    font-size: 40px;
  }
  
  .instruction {
    font-size: 14px;
  }
}
</style> 