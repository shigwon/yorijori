<template>
  <div class="food-compartment-container">
    <!-- 메인 콘텐츠 -->
    <div class="main-content">
      <!-- 제목 텍스트 -->
      <div class="title-text">
        <span class="text-part">음식이 </span>
        <span class="number-part">{{ sectionNum || '?' }}</span>
        <span class="text-part">번째 칸에 있어요</span>
      </div>
      

      
      <!-- 로봇 캐릭터 -->
      <div class="robot-character">
        <img src="../assets/babo.png" alt="Babo Robot" class="babo-image" />
      </div>
    </div>
    
            <!-- 하단 섹션 -->
    <div class="bottom-section">
              <!-- 확인 버튼 -->
      <button class="confirm-button" @click="handleConfirm">
        확인
      </button>
      
              <!-- 설문조사 링크 -->
      <div class="survey-link" @click="handleSurvey">
        고객만족도 평가하러 가기
      </div>
    </div>
  </div>
</template>

<script setup>
import { useAppState } from '../composables/useAppState'

const { closeFoodCompartment, goToSurvey, robotId, sectionNum, orderCode } = useAppState()

const handleConfirm = () => {
  closeFoodCompartment()
  // 페이지 나가기
  window.close()
}

const handleSurvey = () => {
  closeFoodCompartment()
  goToSurvey()
}
</script>

<style scoped>
.food-compartment-container {
  width: 100%;
  height: 100vh;
  height: 100dvh;
  background: #7C3AED;
  display: flex;
  flex-direction: column;
  align-items: center;
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  overflow: hidden;
  z-index: 9999;
}

/* Main Content */
.main-content {
  flex: 1;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: flex-start;
  padding: 60px 20px 0 20px;
  text-align: center;
}

/* Title Text */
.title-text {
  font-size: 24px;
  font-weight: 700;
  color: white;
  margin-bottom: 30px;
  line-height: 1.4;
}

.number-part {
  font-size: 32px;
  font-weight: 900;
  color: white;
  margin: 0 4px;
}

/* 로봇 정보 스타일 */
.robot-info {
  margin-bottom: 20px;
  text-align: center;
}



/* Robot Character */
.robot-character {
  position: relative;
  display: flex;
  flex-direction: column;
  align-items: center;
}

.babo-image {
  width: 240px;
  height: 240px;
  object-fit: contain;
  margin-bottom: 15px;
}

/* Bottom Section */
.bottom-section {
  padding: 30px 20px;
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 16px;
  position: fixed;
  top: 480px;
  left: 0;
  right: 0;
  z-index: 1000;
}

/* Confirm Button */
.confirm-button {
  width: 100%;
  max-width: 300px;
  height: 56px;
  background: white;
  border: none;
  border-radius: 12px;
  font-size: 18px;
  font-weight: 600;
  color: #1F2937;
  cursor: pointer;
  transition: all 0.2s ease;
}

.confirm-button:hover {
  background: #F3F4F6;
  transform: translateY(-1px);
}

.confirm-button:active {
  transform: translateY(0);
}

/* Survey Link */
.survey-link {
  font-size: 14px;
  color: #1F2937;
  cursor: pointer;
  text-decoration: underline;
  transition: color 0.2s ease;
}

.survey-link:hover {
  color: #374151;
}

/* Responsive Design */
@media (max-width: 480px) {
  .main-content {
    padding: 40px 20px 0 20px;
  }
  
  .title-text {
    font-size: 20px;
    margin-bottom: 25px;
  }
  
  .number-part {
    font-size: 28px;
  }
  
  .babo-image {
    width: 200px;
    height: 200px;
  }
  
  .bottom-section {
    top: 420px;
    padding: 25px 20px;
  }
  
  .confirm-button {
    height: 48px;
    font-size: 16px;
  }
  
  .survey-link {
    font-size: 13px;
  }
}
</style> 