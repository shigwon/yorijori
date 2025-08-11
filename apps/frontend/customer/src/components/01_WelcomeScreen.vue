<template>
  <div class="welcome-container">
         <!-- 중앙 로고 -->
     <div class="logo-section">
       <img src="../assets/homerobot.png" alt="lingki logo" class="logo-image" />
     </div>
     
           <!-- 텍스트 섹션 -->
      <div class="text-section">
        <h1 class="brand-title">
          <span class="brand-name">LiNKY</span>
          <span class="brand-subtitle">편하고 빠른 배달부터 배달 정보까지</span>
          <span class="brand-description">지금 내 위치를 설정하고 배송을 받아보세요!</span>
        </h1>
        
        <!-- 로봇 정보 표시 섹션 -->
        <div v-if="robotId || sectionNum" class="robot-info-section">
          <div class="robot-info-card">
            <div class="robot-info-title">📦 배달 정보</div>
            <div v-if="robotId" class="robot-info-item">
              <span class="info-label">로봇 번호:</span>
              <span class="info-value">{{ robotId }}번 로봇</span>
            </div>
            <div v-if="sectionNum" class="robot-info-item">
              <span class="info-label">음식함 번호:</span>
              <span class="info-value">{{ sectionNum }}번 음식함</span>
            </div>
            <div v-if="orderCode" class="robot-info-item">
              <span class="info-label">주문번호:</span>
              <span class="info-value">{{ orderCode }}</span>
            </div>
          </div>
        </div>
      </div>
    
    
    
    <!-- 하단 섹션 (버튼 포함) -->
    <div class="bottom-section">
      <!-- 챗봇 시스템 -->
      <div class="chatbot-system">
                 <!-- 대화창 -->
         <div class="chat-message" v-if="showChatMessage">
           <div class="message-content">
                                                                                                           <div class="message-text">
                  <div>궁금하신게 있으신가요?</div>
                  <div class="message-line">
                    <span>저에게 물어보세요!</span>
                    <img src="../assets/robot.png" alt="robot" class="inline-robot-icon" />
                  </div>
                </div>
             <button class="close-button" @click="hideChatMessage">×</button>
           </div>
           
         </div>
        
                                   <!-- 로봇 아이콘 -->
          <div class="chatbot-icon" @click="openChatbot">
            <img src="../assets/robot.png" alt="robot" class="robot-icon-image" />
          </div>
      </div>
      
             <!-- 시작 버튼 -->
       <button class="start-button" @click="handleStart">
         시작하기
       </button>
     </div>
   </div>
   
   <!-- 챗봇 인터페이스 -->
   <ChatbotInterface v-if="showChatbot" @close="closeChatbot" />
 </template>

<script setup>
import { ref, onMounted } from 'vue'
import { useAppState } from '../composables/useAppState'
import ChatbotInterface from './12_ChatbotInterface.vue'

const { goToHowToUse, openChatbot: openChatbotGlobal, closeChatbot: closeChatbotGlobal, orderCode, robotId, orderId, sectionNum, parseUrlParameters } = useAppState()
const showChatMessage = ref(true)
const showChatbot = ref(false)

// URL에서 모든 파라미터 파싱
onMounted(() => {
  const parsedInfo = parseUrlParameters()
  console.log('파싱된 정보:', parsedInfo)
})

const handleStart = () => {
  goToHowToUse()
}

const toggleChatMessage = () => {
  showChatMessage.value = !showChatMessage.value
}

const hideChatMessage = () => {
  showChatMessage.value = false
}

const openChatbot = () => {
  showChatbot.value = true
  openChatbotGlobal()
}

const closeChatbot = () => {
  showChatbot.value = false
  closeChatbotGlobal()
}
</script>

<style scoped>
.welcome-container {
  min-height: 100vh;
  background: white;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: space-between;
  padding: 60px 24px 40px 24px;
  box-sizing: border-box;
  position: relative;
}

/* Logo Section */
.logo-section {
  position: absolute;
  top: 28%;
  left: 50%;
  transform: translate(-50%, -50%);
  z-index: 1;
  opacity: 1;
}

.logo-image {
  width: 500px;
  height: 500px;
  object-fit: contain;
}

/* 텍스트 섹션 */
.text-section {
  text-align: center;
  width: 100%;
  margin-bottom: 60px;
  flex: 1;
  display: flex;
  align-items: flex-end;
  justify-content: center;
  position: relative;
  z-index: 30;
  bottom: 150px;
}

.brand-title {
  display: flex;
  flex-direction: column;
  gap: 12px;
  margin: 0;
  align-items: center;
}

.brand-name {
  font-size: 32px;
  font-weight: 700;
  background: linear-gradient(135deg, #6D28D9 0%, #5B21B6 100%);
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  background-clip: text;
  line-height: 1.2;
  text-shadow: 2px 2px 4px rgba(255, 255, 255, 0.8);
}

.brand-subtitle {
  font-size: 18px;
  font-weight: 400;
  color: #666;
  line-height: 1.4;
  text-shadow: 1px 1px 3px rgba(255, 255, 255, 0.8);
}

.brand-description {
  font-size: 16px;
  font-weight: 400;
  color: #666;
  line-height: 1.4;
  text-shadow: 1px 1px 3px rgba(255, 255, 255, 0.8);
}

/* 로봇 정보 섹션 스타일 */
.robot-info-section {
  margin-top: 24px;
  width: 100%;
  display: flex;
  justify-content: center;
}

.robot-info-card {
  background: linear-gradient(135deg, #F3F4F6 0%, #E5E7EB 100%);
  border: 2px solid #D1D5DB;
  border-radius: 16px;
  padding: 20px;
  width: 100%;
  max-width: 400px;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.1);
}

.robot-info-title {
  font-size: 18px;
  font-weight: 700;
  color: #374151;
  text-align: center;
  margin-bottom: 16px;
  padding-bottom: 8px;
  border-bottom: 2px solid #D1D5DB;
}

.robot-info-item {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 12px;
  padding: 8px 0;
}

.robot-info-item:last-child {
  margin-bottom: 0;
}

.info-label {
  font-size: 14px;
  font-weight: 600;
  color: #6B7280;
}

.info-value {
  font-size: 16px;
  font-weight: 700;
  color: #1F2937;
  background: linear-gradient(135deg, #7C3AED 0%, #6D28D9 100%);
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  background-clip: text;
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
  height: 56px;
  background: linear-gradient(135deg, #7C3AED 0%, #6D28D9 100%);
  border: none;
  border-radius: 16px;
  color: white;
  font-size: 18px;
  font-weight: 600;
  cursor: pointer;
  box-shadow: 0 4px 20px rgba(124, 60, 237, 0.2);
  z-index: 20;
  position: absolute;
  bottom: 80px;
  left: 50%;
  transform: translateX(-50%);
}

.start-button:hover {
  background: #6D28D9;
  box-shadow: 0 6px 25px rgba(124, 60, 237, 0.3);
}

.start-button:active {
  /* 움직임 제거 */
}

/* Chatbot System */
.chatbot-system {
  position: absolute;
  bottom: 140px;
  right: 24px;
  z-index: 10;
}

/* Chat Message */
.chat-message {
  position: absolute;
  bottom: 40px;
  right: 0;
  background: white;
  border-radius: 10px;
  box-shadow: 0 4px 20px rgba(0, 0, 0, 0.1);
  border: 1px solid rgba(0, 0, 0, 0.05);
  width: 160px;
  z-index: 15;
}



.message-content {
  position: relative;
  padding: 8px 12px;
}

.message-text {
  font-size: 11px;
  line-height: 1.2;
  color: #333;
}

.message-text div:first-child {
  margin-bottom: 4px;
  font-weight: 500;
}

.message-line {
  display: flex;
  align-items: center;
  gap: 6px;
}

.inline-robot-icon {
  width: 12px;
  height: 12px;
  object-fit: contain;
}

.close-button {
  position: absolute;
  top: 4px;
  right: 4px;
  background: none;
  border: none;
  font-size: 14px;
  color: #666;
  cursor: pointer;
  padding: 2px;
  border-radius: 50%;
  width: 18px;
  height: 18px;
  display: flex;
  align-items: center;
  justify-content: center;
  transition: background-color 0.2s;
}

.close-button:hover {
  background-color: rgba(0, 0, 0, 0.05);
}

.chatbot-icon {
  position: absolute;
  bottom: 0;
  right: 0;
  width: 40px;
  height: 40px;
  border-radius: 50%;
  overflow: hidden;
  display: flex;
  align-items: center;
  justify-content: center;
  background: white;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
  cursor: pointer;
  transition: transform 0.2s;
}

.chatbot-icon:hover {
  transform: scale(1.05);
}

.robot-icon-image {
  width: 28px;
  height: 28px;
  object-fit: contain;
}

.chatbot-text {
  font-size: 12px;
  font-weight: 500;
  color: white;
}

/* Responsive Design */
@media (max-width: 768px) {
  .logo-image {
    width: 400px;
    height: 400px;
  }
  
  .brand-name {
    font-size: 28px;
  }
  
  .brand-subtitle {
    font-size: 16px;
  }
  
  .brand-description {
    font-size: 14px;
  }
}

@media (max-width: 480px) {
  .welcome-container {
    padding: 40px 16px 30px 16px;
  }
  
  .logo-image {
    width: 350px;
    height: 350px;
  }
  
  .brand-name {
    font-size: 24px;
  }
  
  .brand-subtitle {
    font-size: 14px;
  }
  
  .brand-description {
    font-size: 12px;
  }
  
  .chatbot-system {
    position: absolute;
    bottom: 140px;
    right: 16px;
    margin-top: 0;
  }
}
</style> 