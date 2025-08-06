<template>
  <div class="modal-overlay" @click="closeModal">
    <div class="modal-content" @click.stop>
      <!-- Title -->
      <div class="title">음식이 도착했어요!</div>
      
      <!-- Robot Character -->
      <div class="robot-character">
        <div class="robot-head">
          <div class="antenna left"></div>
          <div class="antenna right"></div>
          <div class="eyes">
            <div class="eye left"></div>
            <div class="eye right"></div>
          </div>
          <div class="mouth"></div>
        </div>
        <div class="robot-body"></div>
        <div class="robot-shadow"></div>
      </div>
      
      <!-- Instruction Text -->
      <div class="instruction">얼굴을 인식해서 음식을 받아주세요</div>
    </div>
  </div>
</template>

<script setup>
import { defineEmits, onMounted } from 'vue'

const emit = defineEmits(['close', 'show-compartment'])

const closeModal = () => {
  emit('close')
}

onMounted(() => {
  console.log('배달 완료 모달 마운트됨')
  
  // 4초 후 음식 칸 화면으로 전환하고 모달 닫기
  setTimeout(() => {
    console.log('음식 칸 화면으로 전환')
    emit('show-compartment')
    emit('close') // 모달도 함께 닫기
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

.robot-character {
  position: relative;
  display: flex;
  flex-direction: column;
  align-items: center;
  margin-bottom: 30px;
}

.robot-head {
  position: relative;
  width: 80px;
  height: 80px;
  background: #E5E7EB;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  margin-bottom: 15px;
}

.antenna {
  position: absolute;
  top: -8px;
  width: 2px;
  height: 12px;
  background: #6B7280;
  border-radius: 1px;
}

.antenna.left {
  left: 30px;
}

.antenna.right {
  right: 30px;
}

.eyes {
  display: flex;
  gap: 12px;
}

.eye {
  width: 12px;
  height: 12px;
  background: #374151;
  border-radius: 50%;
}

.mouth {
  position: absolute;
  bottom: 20px;
  width: 16px;
  height: 6px;
  border: 2px solid #374151;
  border-top: none;
  border-radius: 0 0 8px 8px;
}

.robot-body {
  width: 60px;
  height: 70px;
  background: linear-gradient(135deg, #60A5FA 0%, #34D399 100%);
  border-radius: 30px 30px 20px 20px;
  position: relative;
}

.robot-shadow {
  width: 50px;
  height: 8px;
  background: rgba(0, 0, 0, 0.1);
  border-radius: 50%;
  margin-top: 8px;
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
  
  .robot-head {
    width: 70px;
    height: 70px;
  }
  
  .robot-body {
    width: 50px;
    height: 60px;
  }
  
  .instruction {
    font-size: 14px;
  }
}
</style> 