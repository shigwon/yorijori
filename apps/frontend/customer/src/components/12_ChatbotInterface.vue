<template>
  <div class="chatbot-interface">
         <!-- 헤더 -->
     <div class="chat-header">
                      <div class="header-left">
           <div class="bot-icon">
             <img src="../assets/robot.png" alt="robot" class="bot-icon-image" />
           </div>
           <span class="bot-name">링키 챗봇</span>
         </div>
       <button class="close-button" @click="closeChat">×</button>
     </div>

     <!-- 날짜 -->
     <div class="date-info">
       - {{ currentDate }} -
     </div>

     <!-- 채팅 영역 -->
     <div class="chat-area" ref="chatArea">
              <!-- 초기 봇 메시지 -->
        <div class="message bot-message">
          <div class="message-content">
            <div class="message-text">
              <div>안녕하세요.</div>
              <div>링키 챗봇 AI 상담서비스입니다.</div>
              <div>어떤 문제로 상담이 필요하신가요?</div>
            </div>
          </div>
        </div>

              <!-- 상담 옵션 -->
        <div class="consultation-options">
          <div class="options-row">
            <button class="option-button" @click="selectOption('수유/이유식')">수유/이유식</button>
            <button class="option-button" @click="selectOption('수면')">수면</button>
            <button class="option-button" @click="selectOption('생활습관/스케줄')">생활습관/스케줄</button>
            <button class="option-button" @click="selectOption('성장/발달')">성장/발달</button>
            <button class="option-button" @click="selectOption('놀이/행동/울음')">놀이/행동/울음</button>
            <button class="option-button" @click="selectOption('기타')">기타</button>
          </div>
        </div>

                                                                                                                   <!-- 채팅 메시지들 -->
                       <div v-for="(message, index) in messages" :key="index" class="message" :class="message.sender === 'bot' ? 'bot-message' : 'user-message'">
              <div class="message-wrapper">
                <div class="message-content">
                  <div v-if="message.sender === 'bot'" class="message-header">
                    <img src="../assets/robot.png" alt="robot" class="message-robot-icon" />
                    <span class="message-bot-name">링키 챗봇</span>
                  </div>
                  <div class="message-text">{{ message.content }}</div>
                </div>
                <div class="message-time">{{ message.timestamp }}</div>
              </div>
            </div>
     </div>

     <!-- 입력 영역 -->
     <div class="input-area">
       <button class="attach-button">+</button>
       <input 
         v-model="userInput" 
         @keyup.enter="sendMessage"
         type="text" 
         placeholder="질문을 입력하세요" 
         class="message-input"
         :disabled="isLoading"
       />
       <button class="send-button" @click="sendMessage" :disabled="isLoading">
         <svg v-if="!isLoading" width="20" height="20" viewBox="0 0 24 24" fill="none">
           <path d="M2.01 21L23 12 2.01 3 2 10l15 2-15 2z" fill="currentColor"/>
         </svg>
         <div v-else class="loading-spinner"></div>
       </button>
     </div>
  </div>
</template>

<script setup>
import { ref, computed, onMounted, nextTick } from 'vue'
import { useAppState } from '../composables/useAppState'
import { useWebSocket } from '../composables/useWebSocket'
import { useAiChat } from '../composables/useAiChat'

const { closeChatbot } = useAppState()
const { connected, messages, sendMessage: sendWebSocketMessage, setOrderCode } = useWebSocket()
const { sendMessage: sendAiMessage, isLoading, error } = useAiChat()

const userInput = ref('')
const chatArea = ref(null)

const currentDate = computed(() => {
  const now = new Date()
  const year = now.getFullYear()
  const month = now.getMonth() + 1
  const day = now.getDate()
  const weekdays = ['일', '월', '화', '수', '목', '금', '토']
  const weekday = weekdays[now.getDay()]
  return `${year}. ${month}. ${day} ${weekday}요일`
})

const getCurrentTime = () => {
  const now = new Date()
  const hours = now.getHours().toString().padStart(2, '0')
  const minutes = now.getMinutes().toString().padStart(2, '0')
  return `${hours}:${minutes}`
}

const selectOption = async (option) => {
   const time = getCurrentTime()
   
   // WebSocket을 통해 사용자 선택 메시지 전송
   sendWebSocketMessage('user', `[${option}] ${option}`)

   // AI API 호출
   try {
     const aiResponse = await sendAiMessage(`${option}에 대한 상담을 받고 싶습니다.`)
     sendWebSocketMessage('bot', aiResponse)
   } catch (err) {
     console.error('AI 응답 실패:', err)
     sendWebSocketMessage('bot', `${option} 상담을 진행하겠습니다. 하단 질문에 내용을 입력해주시길 바랍니다.`)
   }
   
   scrollToBottom()
 }

 const sendMessage = async () => {
  if (!userInput.value.trim()) return

  const time = getCurrentTime()
  const userQuestion = userInput.value
  
  // WebSocket을 통해 사용자 메시지 전송
  sendWebSocketMessage('user', userQuestion)
  
  userInput.value = ''

  // AI API 호출
  try {
    const aiResponse = await sendAiMessage(userQuestion)
    sendWebSocketMessage('bot', aiResponse)
  } catch (err) {
    console.error('AI 응답 실패:', err)
    sendWebSocketMessage('bot', '죄송합니다. AI 서비스에 일시적인 문제가 발생했습니다.')
  }
  
  scrollToBottom()
}



const scrollToBottom = () => {
  nextTick(() => {
    if (chatArea.value) {
      chatArea.value.scrollTop = chatArea.value.scrollHeight
    }
  })
}

const closeChat = () => {
  closeChatbot()
}

onMounted(() => {
  scrollToBottom()
})
</script>

<style scoped>
.chatbot-interface {
  position: fixed;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  background: white;
  display: flex;
  flex-direction: column;
  z-index: 1000;
}

 /* 헤더 */
 .chat-header {
   display: flex;
   justify-content: space-between;
   align-items: center;
   padding: 16px 20px;
   background: white;
   border-bottom: 1px solid #e0e0e0;
 }

 .header-left {
   display: flex;
   align-items: center;
   gap: 12px;
 }

 .bot-icon {
   width: 32px;
   height: 32px;
   border-radius: 50%;
   background: white;
   display: flex;
   align-items: center;
   justify-content: center;
   box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
   overflow: hidden;
 }

 .bot-icon-image {
   width: 24px;
   height: 24px;
   object-fit: contain;
 }

 .bot-name {
   font-size: 16px;
   font-weight: 600;
   color: #333;
 }

 .close-button {
   background: none;
   border: none;
   font-size: 24px;
   color: #666;
   cursor: pointer;
   padding: 4px;
   border-radius: 50%;
   width: 32px;
   height: 32px;
   display: flex;
   align-items: center;
   justify-content: center;
   transition: background-color 0.2s;
 }

 .close-button:hover {
   background-color: rgba(0, 0, 0, 0.05);
 }

 /* 날짜 */
 .date-info {
   text-align: center;
   padding: 8px;
   color: #999;
   font-size: 12px;
   background: #f5f5f5;
   position: relative;
 }

 /* .date-info::after {
   content: '14:31';
   position: absolute;
   left: 16px;
   top: 50%;
   transform: translateY(-50%);
   font-size: 11px;
   color: #999;
 } */

 /* 채팅 영역 */
 .chat-area {
   flex: 1;
   padding: 16px;
   overflow-y: auto;
   background: #f5f5f5;
 }

 .message {
   margin-bottom: 16px;
   max-width: 80%;
   display: flex;
   flex-direction: column;
   position: relative;
   align-items: flex-start;
 }

 .message-wrapper {
   display: flex;
   flex-direction: column;
   align-items: flex-start;
   width: fit-content;
 }

 .bot-message {
   align-self: flex-start;
   margin-right: auto;
 }

 .user-message {
   align-self: flex-end;
   margin-left: auto;
   align-items: flex-end;
 }

 .user-message .message-wrapper {
   align-items: flex-end;
 }

 .user-message .message-content {
   align-self: flex-end;
 }

 .user-message .message-time {
   align-self: flex-end;
   margin-left: auto;
 }

 .message-content {
   padding: 8px 12px;
   border-radius: 16px;
   position: relative;
   box-shadow: 0 1px 3px rgba(0, 0, 0, 0.1);
   display: inline-block;
   max-width: fit-content;
   word-wrap: break-word;
   margin-bottom: 0;
   align-self: flex-start;
   width: fit-content;
 }

 .bot-message .message-content {
   background: white;
   color: #333;
   border: 1px solid #f0f0f0;
   box-shadow: 0 1px 3px rgba(0, 0, 0, 0.1);
 }

 .message-header {
   display: flex;
   align-items: center;
   gap: 8px;
   margin-bottom: 8px;
 }

 .message-robot-icon {
   width: 16px;
   height: 16px;
   object-fit: contain;
 }

 .message-bot-name {
   font-size: 11px;
   font-weight: 600;
   color: #7C3AED;
 }

 .user-message .message-content {
   background: #7C3AED;
   color: white;
 }

 .message-text {
   font-size: 12px;
   line-height: 1.3;
 }

 .message-text div {
   margin-bottom: 4px;
 }

 .message-text div:last-child {
   margin-bottom: 0;
 }

 .message-time {
   font-size: 11px;
   color: #999;
   margin-top: 4px;
   text-align: right;
   white-space: nowrap;
   padding-right: 4px;
   align-self: flex-end;
   max-width: fit-content;
   width: fit-content;
   margin-left: auto;
 }

 /* 상담 옵션 */
 .consultation-options {
   margin: 16px 0;
 }

 .options-row {
   display: flex;
   gap: 8px;
   margin-bottom: 8px;
 }

 .option-button {
   flex: 0 0 auto;
   padding: 8px 12px;
   background: white;
   border: 1px solid #e0e0e0;
   border-radius: 12px;
   color: #7C3AED;
   font-size: 12px;
   cursor: pointer;
   transition: all 0.2s;
   margin-right: 8px;
   margin-bottom: 8px;
   white-space: nowrap;
 }

 .option-button:hover {
   background: #f8f8f8;
   border-color: #7C3AED;
 }

 .option-button.selected {
   background: #7C3AED;
   color: white;
   border-color: #7C3AED;
 }

 /* 입력 영역 */
 .input-area {
   display: flex;
   align-items: center;
   gap: 12px;
   padding: 12px 20px;
   background: white;
   border-top: 1px solid #e0e0e0;
 }

 .attach-button {
   background: none;
   border: none;
   font-size: 20px;
   color: #666;
   cursor: pointer;
   padding: 8px;
   border-radius: 50%;
   width: 36px;
   height: 36px;
   display: flex;
   align-items: center;
   justify-content: center;
   transition: background-color 0.2s;
 }

 .attach-button:hover {
   background-color: rgba(0, 0, 0, 0.05);
 }

 .message-input {
   flex: 1;
   padding: 10px 16px;
   border: 1px solid #e0e0e0;
   border-radius: 20px;
   font-size: 14px;
   outline: none;
   transition: border-color 0.2s;
   background: white;
 }

 .message-input:focus {
   border-color: #7C3AED;
 }

 .message-input::placeholder {
   color: #999;
 }

 .send-button {
   background: #7C3AED;
   border: none;
   border-radius: 50%;
   width: 36px;
   height: 36px;
   display: flex;
   align-items: center;
   justify-content: center;
   cursor: pointer;
   color: white;
   transition: background-color 0.2s;
 }

 .send-button:hover {
   background: #6D28D9;
 }

 .send-button:disabled {
   background: #ccc;
   cursor: not-allowed;
 }

 .loading-spinner {
   width: 16px;
   height: 16px;
   border: 2px solid #ffffff;
   border-top: 2px solid transparent;
   border-radius: 50%;
   animation: spin 1s linear infinite;
 }

 @keyframes spin {
   0% { transform: rotate(0deg); }
   100% { transform: rotate(360deg); }
 }

 /* 반응형 디자인 */
 @media (max-width: 768px) {
   .message {
     max-width: 90%;
   }
   
   .options-row {
     flex-wrap: wrap;
     gap: 6px;
   }
   
   .option-button {
     flex: 0 0 auto;
     padding: 6px 10px;
     font-size: 11px;
     margin-right: 6px;
     margin-bottom: 6px;
   }
 }
</style> 