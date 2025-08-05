<template>
  <div class="welcome-container">
    <!-- 배경 반짝임 효과 -->
    <div class="sparkle sparkle-1"></div>
    <div class="sparkle sparkle-2"></div>
    <div class="sparkle sparkle-3"></div>
    <div class="sparkle sparkle-4"></div>
    
    <!-- 텍스트 섹션 -->
    <div class="text-section">
      <h1 class="brand-title">
        <span class="brand-name">LiNKY 로</span>
        <span class="brand-subtitle">편하고 빠른 배달을 시작해요</span>
      </h1>
    </div>
    
    <!-- 중앙 로고 -->
    <div class="logo-section">
      <img src="../assets/home.png" alt="lingki logo" class="logo-image" />
    </div>
    
    <!-- 하단 섹션 (버튼 포함) -->
    <div class="bottom-section">
      <!-- 챗봇 시스템 -->
      <div class="chatbot-system">
        <div class="chatbot-icon">💬</div>
        <span class="chatbot-text">챗봇 시스템</span>
      </div>
      
      <!-- 시작 버튼 -->
      <button class="start-button" @click="handleStart">
        시작하기
      </button>
    </div>
  </div>
</template>

<script setup>
const emit = defineEmits(['start'])

const handleStart = () => {
  emit('start')
}
</script>

<style scoped>
.welcome-container {
  min-height: 100vh;
  background: #6A0DAD;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: space-between;
  padding: 60px 24px 40px 24px;
  box-sizing: border-box;
  position: relative;
  overflow: hidden;
}

/* 반짝임 효과 */
.sparkle {
  position: absolute;
  width: 12px;
  height: 12px;
  background: white;
  clip-path: polygon(50% 0%, 61% 35%, 98% 35%, 68% 57%, 79% 91%, 50% 70%, 21% 91%, 32% 57%, 2% 35%, 39% 35%);
  opacity: 0.8;
}

.sparkle-1 {
  top: 15%;
  left: 20%;
}

.sparkle-2 {
  top: 20%;
  left: 25%;
}

.sparkle-3 {
  top: 25%;
  right: 25%;
}

.sparkle-4 {
  bottom: 35%;
  right: 20%;
}

/* 텍스트 섹션 */
.text-section {
  text-align: left;
  width: 100%;
  margin-bottom: 40px;
  align-self: flex-start;
}

.brand-title {
  display: flex;
  flex-direction: column;
  gap: 8px;
  margin: 0;
}

.brand-name {
  font-size: 42px;
  font-weight: 700;
  color: rgba(255, 255, 255, 0.9);
  line-height: 1.2;
}

.brand-subtitle {
  font-size: 28px;
  font-weight: 500;
  color: rgba(255, 255, 255, 0.9);
  line-height: 1.4;
}

/* Logo Section */
.logo-section {
  display: flex;
  align-items: center;
  justify-content: center;
  position: absolute;
  top: 40%;
  left: 50%;
  transform: translate(-50%, -50%);
  margin: 0;
}

.logo-image {
  width: 160px;
  height: 160px;
  object-fit: contain;
}

/* Bottom Section */
.bottom-section {
  width: 100%;
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 24px;
  position: relative;
  margin-top: auto;
  padding-bottom: 40px;
}

/* Start Button */
.start-button {
  width: 100%;
  max-width: 320px;
  height: 56px;
  background: white;
  color: #6A0DAD;
  border: 1px solid rgba(255, 255, 255, 0.3);
  border-radius: 12px;
  font-size: 18px;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.3s ease;
  box-shadow: 0 4px 20px rgba(0, 0, 0, 0.1);
  z-index: 20;
  position: absolute;
  bottom: 80px;
  left: 50%;
  transform: translateX(-50%);
}

.start-button:hover {
  transform: translateY(-2px);
  box-shadow: 0 6px 25px rgba(0, 0, 0, 0.15);
}

.start-button:active {
  transform: translateY(0);
}



/* Chatbot System */
.chatbot-system {
  display: flex;
  align-items: center;
  gap: 8px;
  position: absolute;
  bottom: 140px;
  right: 24px;
  padding: 8px 12px;
  background: rgba(59, 130, 246, 0.9);
  border-radius: 16px;
  z-index: 10;
}

.chatbot-icon {
  font-size: 16px;
}

.chatbot-text {
  font-size: 12px;
  font-weight: 500;
  color: white;
}

/* Responsive Design */
@media (max-width: 768px) {
  .logo-image {
    width: 180px;
    height: 180px;
  }
  
  .brand-name {
    font-size: 36px;
  }
  
  .brand-subtitle {
    font-size: 24px;
  }
  
  .bottom-section {
    align-items: flex-start;
  }
  
  .text-section {
    text-align: left;
  }
}

@media (max-width: 480px) {
  .welcome-container {
    padding: 40px 16px 30px 16px;
  }
  
  .logo-image {
    width: 140px;
    height: 140px;
  }
  
  .brand-name {
    font-size: 28px;
  }
  
  .brand-subtitle {
    font-size: 20px;
  }
  
  .chatbot-system {
    position: absolute;
    bottom: 140px;
    right: 16px;
    margin-top: 0;
  }
}
</style> 