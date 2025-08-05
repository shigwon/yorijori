<template>
  <div class="complete-screen-container">
    <!-- 중앙 텍스트 섹션 -->
    <div class="text-section">
      <h1 class="main-title">배송 진행 중입니다</h1>
      <p class="subtitle">고객님께 안전하게 배달 중이에요!</p>
    </div>
    
    <!-- 배송 일러스트레이션 -->
    <div class="illustration-section">
      <img src="../assets/Delivering.png" alt="Delivering" class="delivering-image" />
    </div>
    
    <!-- 하단 버튼 -->
    <div class="button-section">
      <button class="home-button" @click="handleNext">홈으로 나가기</button>
    </div>
  </div>
</template>

<script setup>
import { onMounted } from 'vue'
import { useAppState } from '../composables/useAppState'

const { goToOnboarding, setProgressPercent } = useAppState()

onMounted(() => {
  setProgressPercent(90)
})

const handleNext = () => {
  goToOnboarding()
}
</script>

<style scoped>
.complete-screen-container {
  min-height: 100vh;
  background: #fff;
  display: flex;
  flex-direction: column;
  align-items: center;
  padding: 20px 0;
  position: relative;
}

/* 텍스트 섹션 */
.text-section {
  text-align: center;
  margin-bottom: 60px;
  margin-top: 40px;
}

.main-title {
  font-size: 24px;
  font-weight: 700;
  color: #222;
  margin: 0 0 8px 0;
  line-height: 1.2;
}

.subtitle {
  font-size: 18px;
  font-weight: 500;
  color: #222;
  margin: 0;
  line-height: 1.3;
}

/* 일러스트레이션 섹션 */
.illustration-section {
  flex: 1;
  display: flex;
  align-items: center;
  justify-content: center;
  margin-bottom: 60px;
}

.delivering-image {
  max-width: 80%;
  height: auto;
  object-fit: contain;
}

/* 버튼 섹션 */
.button-section {
  width: 100%;
  padding: 0 20px;
  margin-top: auto;
}

.home-button {
  width: 100%;
  max-width: 300px;
  height: 50px;
  background: #a78bfa;
  color: #fff;
  border: none;
  border-radius: 12px;
  font-size: 18px;
  font-weight: 600;
  cursor: pointer;
  transition: background 0.2s;
  box-shadow: 0 4px 12px rgba(167, 139, 250, 0.3);
}

.home-button:hover {
  background: #7c3aed;
}

.home-button:active {
  transform: translateY(1px);
}

/* 반응형 디자인 */
@media (max-width: 480px) {
  .complete-screen-container {
    padding: 16px 0;
    padding-bottom: 80px;
  }
  
  .text-section {
    margin-top: 30px;
  }
  
  .main-title {
    font-size: 22px;
  }
  
  .subtitle {
    font-size: 16px;
  }
  
  .delivering-image {
    max-width: 70%;
  }
  
  .button-section {
    padding-bottom: 60px;
  }
  
  .home-button {
    height: 48px;
    font-size: 16px;
  }
}

/* 작은 모바일 화면에서 추가 여백 */
@media (max-width: 375px) {
  .complete-screen-container {
    padding-bottom: 70px;
  }
  
  .button-section {
    padding-bottom: 50px;
  }
}

/* 매우 작은 화면에서 추가 여백 */
@media (max-width: 320px) {
  .complete-screen-container {
    padding-bottom: 60px;
  }
  
  .button-section {
    padding-bottom: 40px;
  }
}
</style>
