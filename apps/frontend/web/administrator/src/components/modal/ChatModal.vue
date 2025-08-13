<template>
  <Transition name="modal-fade">
    <div v-if="isVisible" class="modal-overlay" @click="closeModal">
      <Transition name="modal-slide">
        <div class="modal-content" @click.stop>
          <!-- 모달 헤더 -->
          <div class="modal-header">
            <div class="chat-info">
              <div class="chat-id">#{{ orderNumber }}</div>
              <div class="connection-status">
                <v-chip 
                  :color="isConnected ? 'success' : 'error'"
                  size="x-small"
                >
                  {{ isConnected ? '연결됨' : '연결 안됨' }}
                </v-chip>
              </div>
            </div>
            <button class="close-button" @click="closeModal">×</button>
          </div>

          <!-- 채팅 영역 -->
          <div class="chat-area" ref="chatArea">
            <div v-if="loading" class="loading-container">
              <v-progress-circular indeterminate color="#3a57e8"></v-progress-circular>
              <span>채팅방 연결 중...</span>
            </div>
            
            <div v-else-if="messages.length === 0" class="empty-state">
              <v-icon size="48" color="#8a92a6">mdi-chat-outline</v-icon>
              <p>아직 메시지가 없습니다</p>
              <p>첫 번째 메시지를 보내보세요!</p>
            </div>
            
            <div v-else class="messages-container">
              <div 
                v-for="message in messages" 
                :key="message.id"
                class="message"
                :class="message.sender === 'admin' ? 'user-message' : 'assistant-message'"
              >
                <div v-if="message.sender !== 'admin'" class="avatar">
                  <v-icon color="#7c3aed">mdi-account</v-icon>
                </div>
                <div class="message-content">
                  <div v-if="message.sender !== 'admin'" class="message-sender">고객</div>
                  <div class="message-bubble" :class="message.sender === 'admin' ? 'user-bubble' : 'assistant-bubble'">
                    {{ message.content }}
                  </div>
                  <div class="message-time">{{ formatTime(message.timestamp) }}</div>
                </div>
              </div>
            </div>
          </div>

          <!-- 입력 영역 -->
          <div class="input-area">
            <div class="input-container">
              <input 
                type="text" 
                placeholder="메시지를 입력하세요..." 
                class="message-input"
                v-model="messageText"
                @keyup.enter="sendMessage"
                :disabled="!isConnected"
              />
              <button 
                class="send-button" 
                @click="sendMessage"
                :disabled="!isConnected || !messageText.trim()"
              >
                <v-icon size="16">mdi-send</v-icon>
              </button>
            </div>
          </div>
        </div>
      </Transition>
    </div>
  </Transition>
</template>
  
  <script setup>
import { ref, watch, onMounted, onUnmounted, nextTick } from 'vue'
import { defineProps, defineEmits } from 'vue'

const props = defineProps({
  isVisible: {
    type: Boolean,
    default: false
  },
  orderNumber: {
    type: String,
    default: ''
  }
})

const emit = defineEmits(['close'])

// Reactive data
const messageText = ref('')
const messages = ref([])
const loading = ref(false)
const isConnected = ref(false)
const chatArea = ref(null)

// WebSocket 관련
let stompClient = null
let subscription = null

// Methods
const closeModal = () => {
  disconnectWebSocket()
  emit('close')
}

const connectWebSocket = () => {
  if (!props.orderNumber) return
  
  try {
    loading.value = true
    console.log('WebSocket 연결 시작:', props.orderNumber)
    
    // SockJS와 STOMP 클라이언트 생성
    const socket = new SockJS('ws://localhost:8080/ws/ws-chat/websocket')
    stompClient = Stomp.over(socket)
    
    // 연결 설정
    stompClient.connect({}, 
      (frame) => {
        console.log('WebSocket 연결 성공:', frame)
        isConnected.value = true
        loading.value = false
        
        // 채팅방 구독
        subscribeToChat()
      },
      (error) => {
        console.error('WebSocket 연결 실패:', error)
        isConnected.value = false
        loading.value = false
      }
    )
  } catch (error) {
    console.error('WebSocket 연결 오류:', error)
    loading.value = false
  }
}

const subscribeToChat = () => {
  if (!stompClient || !props.orderNumber) return
  
  const topic = `/topic/inquiries/${props.orderNumber}`
  console.log('채팅방 구독:', topic)
  
  subscription = stompClient.subscribe(topic, (message) => {
    try {
      const receivedMessage = JSON.parse(message.body)
      console.log('메시지 수신:', receivedMessage)
      
      // 메시지 추가
      messages.value.push({
        id: Date.now() + Math.random(),
        sender: receivedMessage.sender === 'admin' ? 'admin' : 'customer',
        content: receivedMessage.content,
        timestamp: receivedMessage.timestamp || new Date().toISOString()
      })
      
      // 스크롤을 맨 아래로
      nextTick(() => {
        scrollToBottom()
      })
    } catch (error) {
      console.error('메시지 파싱 오류:', error)
    }
  })
}

const sendMessage = () => {
  if (!messageText.value.trim() || !isConnected.value || !props.orderNumber) return
  
  try {
    const message = {
      sender: 'admin',
      content: messageText.value.trim(),
      timestamp: new Date().toISOString()
    }
    
    console.log('메시지 전송:', message)
    
    // WebSocket을 통해 메시지 전송
    stompClient.send(`/app/inquiry/${props.orderNumber}/send`, {}, JSON.stringify(message))
    
    // 로컬 메시지 추가
    messages.value.push({
      id: Date.now() + Math.random(),
      sender: 'admin',
      content: messageText.value.trim(),
      timestamp: new Date().toISOString()
    })
    
    messageText.value = ''
    
    // 스크롤을 맨 아래로
    nextTick(() => {
      scrollToBottom()
    })
  } catch (error) {
    console.error('메시지 전송 오류:', error)
  }
}

const disconnectWebSocket = () => {
  if (subscription) {
    subscription.unsubscribe()
    subscription = null
  }
  
  if (stompClient) {
    stompClient.disconnect()
    stompClient = null
  }
  
  isConnected.value = false
  messages.value = []
  console.log('WebSocket 연결 해제')
}

const scrollToBottom = () => {
  if (chatArea.value) {
    chatArea.value.scrollTop = chatArea.value.scrollHeight
  }
}

const formatTime = (timestamp) => {
  if (!timestamp) return ''
  
  const date = new Date(timestamp)
  const now = new Date()
  const diffMs = now - date
  const diffMins = Math.floor(diffMs / (1000 * 60))
  
  if (diffMins < 1) return '방금 전'
  if (diffMins < 60) return `${diffMins}분 전`
  
  return date.toLocaleTimeString('ko-KR', {
    hour: '2-digit',
    minute: '2-digit'
  })
}

// Watch for modal visibility changes
watch(() => props.isVisible, (newValue) => {
  if (newValue && props.orderNumber) {
    connectWebSocket()
  } else {
    disconnectWebSocket()
  }
})

// Watch for orderNumber changes
watch(() => props.orderNumber, (newValue) => {
  if (props.isVisible && newValue) {
    disconnectWebSocket()
    connectWebSocket()
  }
})

// Lifecycle
onUnmounted(() => {
  disconnectWebSocket()
})
</script>
  
  <style scoped>
  .modal-overlay {
    position: fixed;
    top: 0;
    left: 0;
    width: 100%;
    height: 100%;
    background-color: rgba(0, 0, 0, 0.7);
    display: flex;
    justify-content: center;
    align-items: center;
    z-index: 1000;
  }
  
  .modal-content {
    background-color: #1f2937;
    border-radius: 12px;
    width: 90%;
    max-width: 500px;
    height: 80%;
    max-height: 600px;
    display: flex;
    flex-direction: column;
    overflow: hidden;
  }
  
  .modal-header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 16px 20px;
    border-bottom: 1px solid #374151;
  }
  
  .chat-info {
    display: flex;
    align-items: center;
    gap: 12px;
  }
  
  .chat-id {
    background-color: #7c3aed;
    color: white;
    padding: 4px 12px;
    border-radius: 20px;
    font-size: 14px;
    font-weight: 500;
  }
  
  .connection-status {
    display: flex;
    align-items: center;
  }
  
  .close-button {
    background: none;
    border: none;
    color: #9ca3af;
    font-size: 24px;
    cursor: pointer;
    padding: 0;
    width: 30px;
    height: 30px;
    display: flex;
    align-items: center;
    justify-content: center;
    border-radius: 50%;
    transition: background-color 0.3s;
  }
  
  .close-button:hover {
    background-color: #374151;
  }
  
  .chat-area {
    flex: 1;
    padding: 20px;
    overflow-y: auto;
    display: flex;
    flex-direction: column;
  }
  
  .loading-container {
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    height: 100%;
    gap: 12px;
    color: #8a92a6;
  }
  
  .empty-state {
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    height: 100%;
    color: #8a92a6;
    text-align: center;
  }
  
  .empty-state p {
    margin: 8px 0;
    font-size: 14px;
  }
  
  .messages-container {
    display: flex;
    flex-direction: column;
    gap: 16px;
  }
  
  .message {
    display: flex;
    gap: 12px;
  }
  
  .assistant-message {
    align-items: flex-start;
  }
  
  .user-message {
    justify-content: flex-end;
  }
  
  .avatar {
    width: 40px;
    height: 40px;
    border-radius: 50%;
    background-color: #7c3aed;
    display: flex;
    align-items: center;
    justify-content: center;
    flex-shrink: 0;
  }
  
  .avatar img {
    width: 100%;
    height: 100%;
    border-radius: 50%;
    object-fit: cover;
  }
  
  .message-content {
    display: flex;
    flex-direction: column;
    gap: 4px;
    max-width: 70%;
  }
  
  .message-sender {
    font-size: 12px;
    color: #9ca3af;
    font-weight: 500;
  }
  
  .message-bubble {
    padding: 12px 16px;
    border-radius: 18px;
    font-size: 14px;
    line-height: 1.4;
    word-wrap: break-word;
  }
  
  .assistant-bubble {
    background-color: #374151;
    color: white;
    border-bottom-left-radius: 4px;
  }
  
  .user-bubble {
    background-color: #3b82f6;
    color: white;
    border-bottom-right-radius: 4px;
  }
  
  .message-time {
    font-size: 11px;
    color: #6b7280;
    margin-top: 2px;
  }
  
  .user-message .message-time {
    text-align: right;
  }
  

  
  .input-area {
    padding: 16px 20px;
    border-top: 1px solid #374151;
  }
  
  .input-container {
    display: flex;
    align-items: center;
    gap: 12px;
    background-color: #374151;
    border-radius: 24px;
    padding: 8px 16px;
  }
  
  .message-input {
    flex: 1;
    background: none;
    border: none;
    color: white;
    font-size: 14px;
    outline: none;
    padding: 8px 0;
  }
  
  .message-input::placeholder {
    color: #9ca3af;
  }
  
  .message-input:disabled {
    color: #6b7280;
    cursor: not-allowed;
  }
  
  .send-button {
    background-color: #7c3aed;
    border: none;
    color: white;
    width: 32px;
    height: 32px;
    border-radius: 50%;
    cursor: pointer;
    display: flex;
    align-items: center;
    justify-content: center;
    transition: background-color 0.3s;
  }
  
  .send-button:hover:not(:disabled) {
    background-color: #8b5cf6;
  }
  
  .send-button:disabled {
    background-color: #6b7280;
    cursor: not-allowed;
  }
  
  /* 스크롤바 스타일링 */
  .chat-area::-webkit-scrollbar {
    width: 6px;
  }
  
  .chat-area::-webkit-scrollbar-track {
    background: #374151;
    border-radius: 3px;
  }
  
  .chat-area::-webkit-scrollbar-thumb {
    background: #6b7280;
    border-radius: 3px;
  }
  
  .chat-area::-webkit-scrollbar-thumb:hover {
    background: #9ca3af;
  }
  
  /* 모달 애니메이션 */
  .modal-fade-enter-active,
  .modal-fade-leave-active {
    transition: opacity 0.3s ease;
  }
  
  .modal-fade-enter-from,
  .modal-fade-leave-to {
    opacity: 0;
  }
  
  .modal-slide-enter-active,
  .modal-slide-leave-active {
    transition: all 0.3s ease;
  }
  
  .modal-slide-enter-from {
    opacity: 0;
    transform: scale(0.8) translateY(-50px);
  }
  
  .modal-slide-leave-to {
    opacity: 0;
    transform: scale(0.8) translateY(50px);
  }
  

  </style> 