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
              
              <!-- 배달 정보 표시 -->
              <div v-if="robotId || sectionNum || orderCode" class="delivery-info-message">
                <div class="delivery-info-title">📦 현재 배달 정보</div>
                <div v-if="robotId" class="delivery-info-item">🤖 {{ robotId }}번 로봇이 배달 중</div>
                <div v-if="sectionNum" class="delivery-info-item">📦 {{ sectionNum }}번 음식함으로 이동</div>
                <div v-if="orderCode" class="delivery-info-item">📋 주문번호: {{ orderCode }}</div>
              </div>
              
              <div class="help-text">
                <div>다음과 같은 질문을 할 수 있습니다:</div>
                <div>• "몇 번 로봇이 배달하나요?"</div>
                <div>• "음식은 어디에 있나요?"</div>
                <div>• "주문번호가 뭔가요?"</div>
                <div>• "배달 상황이 어때요?"</div>
              </div>
            </div>
          </div>
        </div>

              <!-- 상담 옵션 -->
        <div class="consultation-options">
          <div class="options-row">
            <button class="option-button" @click="selectOption('로봇 정보')">로봇 정보</button>
            <button class="option-button" @click="selectOption('음식함 위치')">음식함 위치</button>
            <button class="option-button" @click="selectOption('주문 확인')">주문 확인</button>
            <button class="option-button" @click="selectOption('배달 상황')">배달 상황</button>
            <button class="option-button" @click="selectOption('도움말')">도움말</button>
            <button class="option-button" @click="selectOption('기타 문의')">기타 문의</button>
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

const { closeChatbot, goToWelcome, robotId, sectionNum, orderCode } = useAppState()
const { connected, messages, sendMessage: sendWebSocketMessage, setOrderCode } = useWebSocket()
const { sendMessage: sendAiMessage, isLoading, error } = useAiChat()

const userInput = ref('')
const chatArea = ref(null)

// 스마트 답변을 위한 키워드 매핑
const smartResponses = {
  '로봇': () => robotId.value ? `${robotId.value}번 로봇이 배달하고 있습니다.` : '로봇 정보를 확인할 수 없습니다.',
  '음식함': () => sectionNum.value ? `${sectionNum.value}번 음식함에 음식이 담겨있습니다.` : '음식함 번호를 확인할 수 없습니다.',
  '주문번호': () => orderCode.value ? `주문번호는 ${orderCode.value}입니다.` : '주문번호를 확인할 수 없습니다.',
  '배달': () => {
    let response = '현재 배달 진행 상황입니다.\n'
    if (robotId.value) response += `• ${robotId.value}번 로봇이 배달 중\n`
    if (sectionNum.value) response += `• ${sectionNum.value}번 음식함으로 이동 중\n`
    if (orderCode.value) response += `• 주문번호: ${orderCode.value}`
    return response
  }
}

// 스마트 답변 생성 함수
const generateSmartResponse = (userQuestion) => {
  const question = userQuestion.toLowerCase()
  
  // 키워드 매칭
  for (const [keyword, responseFunc] of Object.entries(smartResponses)) {
    if (question.includes(keyword)) {
      return responseFunc()
    }
  }
  
  // 기본 답변
  return null
}

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

   // 스마트 답변 시도
   const smartResponse = generateSmartResponse(option)
   
   if (smartResponse) {
     // 스마트 답변이 있으면 즉시 응답
     sendWebSocketMessage('bot', smartResponse)
   } else {
     // 스마트 답변이 없으면 AI API 호출
     try {
       const aiResponse = await sendAiMessage(`${option}에 대한 상담을 받고 싶습니다.`)
       sendWebSocketMessage('bot', aiResponse)
     } catch (err) {
       console.error('AI 응답 실패:', err)
       sendWebSocketMessage('bot', `${option} 상담을 진행하겠습니다. 하단 질문에 내용을 입력해주시길 바랍니다.`)
     }
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

  // 먼저 스마트 답변 시도
  const smartResponse = generateSmartResponse(userQuestion)
  
  if (smartResponse) {
    // 스마트 답변이 있으면 즉시 응답
    sendWebSocketMessage('bot', smartResponse)
  } else {
    // 스마트 답변이 없으면 AI API 호출
    try {
      const aiResponse = await sendAiMessage(userQuestion)
      sendWebSocketMessage('bot', aiResponse)
    } catch (err) {
      console.error('AI 응답 실패:', err)
      sendWebSocketMessage('bot', '죄송합니다. AI 서비스에 일시적인 문제가 발생했습니다.')
    }
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
  goToWelcome()
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
   background: white;
   padding: 12px 16px;
   border-radius: 12px;
   box-shadow: 0 2px 8px rgba(0, 0, 0, 0.1);
   line-height: 1.5;
   color: #333;
   font-size: 14px;
 }

/* 배달 정보 메시지 스타일 */
.delivery-info-message {
  margin-top: 16px;
  padding: 16px;
  background: linear-gradient(135deg, #F3F4F6 0%, #E5E7EB 100%);
  border-radius: 12px;
  border: 1px solid #D1D5DB;
}

.delivery-info-title {
  font-size: 16px;
  font-weight: 700;
  color: #374151;
  margin-bottom: 12px;
  text-align: center;
}

.delivery-info-item {
  font-size: 14px;
  color: #4B5563;
  margin-bottom: 8px;
  padding: 4px 0;
}

.delivery-info-item:last-child {
  margin-bottom: 0;
}

/* 도움말 텍스트 스타일 */
.help-text {
  margin-top: 16px;
  padding: 12px;
  background: #FEF3C7;
  border-radius: 8px;
  border-left: 4px solid #F59E0B;
}

.help-text > div {
  margin-bottom: 4px;
  font-size: 13px;
  color: #92400E;
}

.help-text > div:last-child {
  margin-bottom: 0;
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