<template>
    <Transition name="modal-fade">
      <div v-if="isVisible" class="modal-overlay" @click="closeModal">
        <Transition name="modal-slide">
          <div class="modal-content" @click.stop>
            <!-- 모달 헤더 -->
            <div class="modal-header">
              <div class="chat-id">#{{ orderNumber }}</div>
              <button class="close-button" @click="closeModal">×</button>
            </div>
  
            <!-- 채팅 영역 -->
            <div class="chat-area">
              <!-- 어시스턴트 메시지 -->
              <div class="message assistant-message">
                <div class="avatar">
                  <!-- <img src="./assistant-avatar.png" alt="Assistant" /> -->
                </div>
                <div class="message-content">
                  <div class="message-sender">Assistant</div>
                  <div class="message-bubble assistant-bubble">
                    I'm doing well, thank you! How can I help you today?
                  </div>
                  <div class="message-time">08:16 AM</div>
                </div>
              </div>
  
              <!-- 사용자 메시지 -->
              <div class="message user-message">
                <div class="message-content">
                  <div class="message-bubble user-bubble">
                    Hello, how are you doing?
                  </div>
                  <div class="message-time">08:15 AM</div>
                </div>
              </div>
  
              <!-- 사용자 메시지 2 -->
              <div class="message user-message">
                <div class="message-content">
                  <div class="message-bubble user-bubble">
                    I have a question about the return policy for a product I purchased.
                  </div>
                  <div class="message-time">Just Now</div>
                </div>
              </div>
  
              <!-- 어시스턴트 타이핑 -->
              <div class="message assistant-message">
                <div class="avatar">
                  <!-- <img src="./assistant-avatar.png" alt="Assistant" /> -->
                </div>
                <div class="message-content">
                  <div class="message-sender">Assistant</div>
                  <div class="message-bubble assistant-bubble typing">
                    <div class="typing-dots">
                      <span></span>
                      <span></span>
                      <span></span>
                    </div>
                  </div>
                </div>
              </div>
            </div>
  
            <!-- 입력 영역 -->
            <div class="input-area">
              <div class="input-container">
                <button class="emoji-button">😊</button>
                <input 
                  type="text" 
                  placeholder="Reply ..." 
                  class="message-input"
                  v-model="messageText"
                  @keyup.enter="sendMessage"
                />
                <button class="send-button" @click="sendMessage">
                  <span class="send-icon">→</span>
                </button>
              </div>
            </div>
          </div>
        </Transition>
      </div>
    </Transition>
  </template>
  
  <script setup>
  import { ref } from 'vue'
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
  
  // Methods
  const closeModal = () => {
    emit('close')
  }
  
  const sendMessage = () => {
    if (messageText.value.trim()) {
      // 메시지 전송 로직
      console.log('메시지 전송:', messageText.value)
      messageText.value = ''
    }
  }
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
  
  .chat-id {
    background-color: #7c3aed;
    color: white;
    padding: 4px 12px;
    border-radius: 20px;
    font-size: 14px;
    font-weight: 500;
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
  
  .typing-dots {
    display: flex;
    gap: 4px;
    align-items: center;
  }
  
  .typing-dots span {
    width: 6px;
    height: 6px;
    background-color: #9ca3af;
    border-radius: 50%;
    animation: typing 1.4s infinite ease-in-out;
  }
  
  .typing-dots span:nth-child(1) {
    animation-delay: -0.32s;
  }
  
  .typing-dots span:nth-child(2) {
    animation-delay: -0.16s;
  }
  
  @keyframes typing {
    0%, 80%, 100% {
      transform: scale(0.8);
      opacity: 0.5;
    }
    40% {
      transform: scale(1);
      opacity: 1;
    }
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
  
  .emoji-button {
    background: none;
    border: none;
    font-size: 18px;
    cursor: pointer;
    padding: 4px;
    border-radius: 50%;
    transition: background-color 0.3s;
  }
  
  .emoji-button:hover {
    background-color: #4b5563;
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
  
  .send-button:hover {
    background-color: #8b5cf6;
  }
  
  .send-icon {
    font-size: 14px;
    font-weight: bold;
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
  
  /* 버튼 호버 효과 */
  .chat-button {
    transition: all 0.2s ease;
    transform: translateY(0);
  }
  
  .chat-button:hover {
    transform: translateY(-2px);
    box-shadow: 0 4px 12px rgba(0, 0, 0, 0.3);
  }
  
  .chat-button:active {
    transform: translateY(0);
  }
  
  /* 모달 내부 요소 애니메이션 */
  .message {
    animation: slideIn 0.5s ease forwards;
    opacity: 0;
    transform: translateY(20px);
  }
  
  .message:nth-child(1) { animation-delay: 0.1s; }
  .message:nth-child(2) { animation-delay: 0.2s; }
  .message:nth-child(3) { animation-delay: 0.3s; }
  .message:nth-child(4) { animation-delay: 0.4s; }
  
  @keyframes slideIn {
    to {
      opacity: 1;
      transform: translateY(0);
    }
  }
  
  /* 입력 필드 포커스 효과 */
  .message-input:focus {
    transform: scale(1.02);
    transition: transform 0.2s ease;
  }
  
  /* 전송 버튼 클릭 효과 */
  .send-button:active {
    transform: scale(0.95);
    transition: transform 0.1s ease;
  }
  </style> 