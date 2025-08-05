<template>
  <div class="survey-container">
    <!-- 로봇 캐릭터 -->
    <div class="robot-character">
      <div class="robot-head">
        <div class="antenna left"></div>
        <div class="antenna right"></div>
        <div class="eyes">
          <div class="eye left">
            <div class="pupil"></div>
          </div>
          <div class="eye right">
            <div class="pupil"></div>
          </div>
        </div>
        <div class="mouth"></div>
      </div>
      <div class="robot-body">
        <div class="arm left"></div>
        <div class="arm right"></div>
      </div>
      <div class="robot-shadow"></div>
    </div>

    <!-- 설문조사 콘텐츠 -->
    <div class="survey-content">
              <!-- 질문 -->
      <div class="question">배송은 어떠셨나요</div>
      
              <!-- 별점 평가 -->
      <div class="star-rating">
        <div 
          v-for="star in 5" 
          :key="star"
          class="star"
          :class="{ 'filled': star <= selectedRating }"
          @click="selectedRating = star"
        >
          ★
        </div>
      </div>
      
              <!-- 리뷰 입력 -->
      <div class="review-section">
        <div class="review-input">
          <div class="placeholder-text">
            배송에 대한 솔직한 리뷰를 남겨주세요
          </div>
          <div class="placeholder-text">
            남겨주신 의견 서비스 품질에 개선됩니다.
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref } from 'vue'

const selectedRating = ref(0)
</script>

<style scoped>
.survey-container {
  width: 100%;
  height: 100vh;
  height: 100dvh;
  background: linear-gradient(135deg, #7C3AED 0%, #EC4899 100%);
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
  padding: 40px 20px;
}

/* Robot Character */
.robot-character {
  position: relative;
  display: flex;
  flex-direction: column;
  align-items: center;
  margin-bottom: 40px;
}

.robot-head {
  position: relative;
  width: 120px;
  height: 120px;
  background: linear-gradient(135deg, #60A5FA 0%, #34D399 100%);
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  margin-bottom: 30px;
}

.antenna {
  position: absolute;
  top: -15px;
  width: 4px;
  height: 20px;
  background: #374151;
  border-radius: 2px;
}

.antenna.left {
  left: 40px;
}

.antenna.right {
  right: 40px;
}

.eyes {
  display: flex;
  gap: 20px;
}

.eye {
  width: 20px;
  height: 20px;
  background: #374151;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
}

.pupil {
  width: 8px;
  height: 8px;
  background: white;
  border-radius: 50%;
}

.mouth {
  position: absolute;
  bottom: 30px;
  width: 24px;
  height: 10px;
  border: 3px solid #374151;
  border-top: none;
  border-radius: 0 0 12px 12px;
}

.robot-body {
  position: relative;
  width: 100px;
  height: 120px;
  background: linear-gradient(135deg, #60A5FA 0%, #34D399 100%);
  border-radius: 50px 50px 30px 30px;
  margin-bottom: 20px;
}

.arm {
  position: absolute;
  width: 24px;
  height: 24px;
  background: linear-gradient(135deg, #60A5FA 0%, #34D399 100%);
  border-radius: 50%;
  top: 50%;
  transform: translateY(-50%);
}

.arm.left {
  left: -12px;
}

.arm.right {
  right: -12px;
}

.robot-shadow {
  width: 100px;
  height: 15px;
  background: rgba(255, 255, 255, 0.3);
  border-radius: 50%;
  margin-top: 15px;
}

/* Survey Content */
.survey-content {
  flex: 1;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  width: 100%;
  max-width: 400px;
}

.question {
  font-size: 24px;
  font-weight: 700;
  color: white;
  margin-bottom: 30px;
  text-align: center;
}

/* Star Rating */
.star-rating {
  display: flex;
  gap: 10px;
  margin-bottom: 40px;
}

.star {
  font-size: 40px;
  color: transparent;
  border: 2px solid white;
  border-radius: 50%;
  width: 50px;
  height: 50px;
  display: flex;
  align-items: center;
  justify-content: center;
  cursor: pointer;
  transition: all 0.2s ease;
  background: transparent;
}

.star.filled {
  color: #F59E0B;
  background: #F59E0B;
  border-color: #F59E0B;
}

.star:hover {
  transform: scale(1.1);
}

/* Review Section */
.review-section {
  width: 100%;
  background: white;
  border-radius: 12px;
  padding: 20px;
  min-height: 120px;
  display: flex;
  align-items: center;
  justify-content: center;
}

.review-input {
  width: 100%;
  text-align: center;
}

.placeholder-text {
  color: #6B7280;
  font-size: 14px;
  line-height: 1.5;
  margin-bottom: 8px;
}

.placeholder-text:last-child {
  margin-bottom: 0;
}

/* Responsive Design */
@media (max-width: 480px) {
  .survey-container {
    padding: 20px 15px;
  }
  
  .robot-head {
    width: 100px;
    height: 100px;
  }
  
  .antenna.left {
    left: 32px;
  }
  
  .antenna.right {
    right: 32px;
  }
  
  .eyes {
    gap: 16px;
  }
  
  .eye {
    width: 18px;
    height: 18px;
  }
  
  .pupil {
    width: 7px;
    height: 7px;
  }
  
  .mouth {
    bottom: 25px;
    width: 20px;
    height: 8px;
  }
  
  .robot-body {
    width: 80px;
    height: 100px;
  }
  
  .arm {
    width: 20px;
    height: 20px;
  }
  
  .question {
    font-size: 20px;
    margin-bottom: 25px;
  }
  
  .star {
    font-size: 32px;
    width: 40px;
    height: 40px;
  }
  
  .review-section {
    padding: 15px;
    min-height: 100px;
  }
  
  .placeholder-text {
    font-size: 13px;
  }
}
</style> 