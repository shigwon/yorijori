<template>
  <div class="sse-test-page">
    <div class="header">
      <h1>SSE 스트리밍 테스트</h1>
      <div class="controls">
        <v-btn 
          @click="startStreaming" 
          :disabled="isConnected"
          color="primary"
          class="control-btn"
        >
          <v-icon>mdi-play</v-icon>
          스트리밍 시작
        </v-btn>
        <v-btn 
          @click="stopStreaming" 
          :disabled="!isConnected"
          color="error"
          class="control-btn"
        >
          <v-icon>mdi-stop</v-icon>
          스트리밍 중지
        </v-btn>
        <v-btn 
          @click="clearMessages" 
          color="warning"
          class="control-btn"
        >
          <v-icon>mdi-delete</v-icon>
          메시지 지우기
        </v-btn>
      </div>
    </div>

    <div class="status-panel">
      <div class="status-item">
        <span class="status-label">연결 상태:</span>
        <v-chip 
          :color="isConnected ? 'success' : 'error'"
          size="small"
        >
          {{ isConnected ? '연결됨' : '연결 안됨' }}
        </v-chip>
      </div>
      <div class="status-item">
        <span class="status-label">메시지 수:</span>
        <span class="status-value">{{ messages.length }}</span>
      </div>
      <div class="status-item">
        <span class="status-label">마지막 업데이트:</span>
        <span class="status-value">{{ lastUpdateTime }}</span>
      </div>
    </div>

    <div class="content-area">
      <div class="streaming-section">
        <h3>실시간 스트리밍 데이터</h3>
        <div class="streaming-container">
          <!-- 일반 메시지 섹션 -->
          <div v-if="generalMessages.length > 0" class="message-section">
            <h4 class="section-title">일반 메시지</h4>
            <div 
              v-for="(message, index) in generalMessages" 
              :key="`general-${index}`"
              class="message-item"
              :class="message.event"
            >
              <div class="message-header">
                <span class="event-type">{{ message.event }}</span>
                <span class="timestamp">{{ formatTime(message.timestamp) }}</span>
              </div>
              <div class="message-content">
                <pre>{{ message.data }}</pre>
              </div>
            </div>
          </div>

                     <!-- 로봇 스트리밍 메시지 섹션 -->
           <div v-if="robotStreamingMessages.length > 0" class="message-section">
             <h4 class="section-title">로봇 스트리밍 데이터</h4>
             
             <!-- 실시간 이미지 표시 -->
             <div v-if="currentImage" class="image-display-section">
               <h5 class="image-title">실시간 로봇 이미지</h5>
               <div class="image-container">
                 <img 
                   :src="currentImage" 
                   alt="로봇 스트리밍 이미지" 
                   class="streaming-image"
                 />
                 <div class="image-info">
                   <span class="image-timestamp">수신 시간: {{ formatTime(imageTimestamp) }}</span>
                 </div>
               </div>
             </div>
             
             <div 
               v-for="(message, index) in robotStreamingMessages" 
               :key="`robot-${index}`"
               class="message-item"
               :class="message.event"
             >
               <div class="message-header">
                 <span class="event-type">{{ message.event }}</span>
                 <span class="timestamp">{{ formatTime(message.timestamp) }}</span>
               </div>
               <div class="message-content">
                 <pre>{{ message.data }}</pre>
               </div>
             </div>
           </div>

          <!-- 메시지가 없을 때 -->
          <div v-if="messages.length === 0" class="no-messages">
            <v-icon size="48" color="#8a92a6">mdi-message-text</v-icon>
            <p>스트리밍 데이터가 없습니다.</p>
            <p>스트리밍을 시작해주세요.</p>
          </div>
        </div>
      </div>

      <div class="settings-section">
        <h3>설정</h3>
        <div class="settings-form">
          <v-text-field
            v-model="robotId"
            label="로봇 ID"
            variant="outlined"
            density="compact"
            placeholder="예: 1"
            class="setting-input"
          ></v-text-field>
          
          <v-text-field
            v-model="streamUrl"
            label="스트리밍 URL"
            variant="outlined"
            density="compact"
            placeholder="스트리밍 엔드포인트"
            class="setting-input"
          ></v-text-field>

          <v-switch
            v-model="autoReconnect"
            label="자동 재연결"
            color="primary"
            class="setting-switch"
          ></v-switch>

          <v-switch
            v-model="showTimestamp"
            label="타임스탬프 표시"
            color="primary"
            class="setting-switch"
          ></v-switch>
        </div>
      </div>
    </div>

    <div class="log-section">
      <h3>연결 로그</h3>
      <div class="log-container">
        <div 
          v-for="(log, index) in connectionLogs" 
          :key="index"
          class="log-item"
          :class="log.type"
        >
          <span class="log-time">{{ formatTime(log.timestamp) }}</span>
          <span class="log-message">{{ log.message }}</span>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, computed, onMounted, onUnmounted } from 'vue'



// Reactive data
const isConnected = ref(false)
const messages = ref([])
const connectionLogs = ref([])
const lastUpdateTime = ref('없음')
const currentImage = ref(null)
const imageTimestamp = ref(null)

// Settings
const robotId = ref('1')
const streamUrl = ref('/api/v1/streaming/subscribe/1')
const autoReconnect = ref(true)
const showTimestamp = ref(true)

// Computed properties for message filtering
const generalMessages = computed(() => {
  return messages.value.filter(msg => msg.event === 'message')
})

const robotStreamingMessages = computed(() => {
  return messages.value.filter(msg => msg.event === 'robotStreamingImage')
})

// SSE 관련
let eventSource = null
let reconnectTimeout = null
let reconnectAttempts = 0
const maxReconnectAttempts = 5

// Methods
const startStreaming = async () => {
  try {
    addLog('info', '스트리밍 시작 시도...')
    
    // 기존 연결이 있으면 닫기
    if (eventSource) {
      eventSource.close()
    }

    // URL 구성
    const url = streamUrl.value || `${ws_BASE_URL}/api/v1/streaming/subscribe/${robotId.value}`
    
    // EventSource 생성
    eventSource = new EventSource(url)
    
          // 연결 성공 - 연결이 성공했을때만 실행됨
      eventSource.onopen = (event) => {
        isConnected.value = true
        reconnectAttempts = 0
        addLog('success', 'SSE 연결 성공')
        console.log('SSE 연결 성공')
        updateLastUpdateTime()
        console.log('업데이트 시간:', lastUpdateTime.value)
      }

    // 메시지 수신
    eventSource.onmessage = (event) => {
      handleMessage(event)
    }

    // 특정 이벤트 수신 (robotStreamingImage)
    eventSource.addEventListener('robotStreamingImage', (event) => {
      handleRobotStreamingImage(event)
    })

    // 에러 처리
    eventSource.onerror = (event) => {
      isConnected.value = false
      addLog('error', 'SSE 연결 오류')
      
      if (autoReconnect.value && reconnectAttempts < maxReconnectAttempts) {
        reconnectAttempts++
        addLog('warning', `재연결 시도 ${reconnectAttempts}/${maxReconnectAttempts}`)
        
        reconnectTimeout = setTimeout(() => {
          startStreaming()
        }, 2000 * reconnectAttempts) // 지수 백오프
      } else if (reconnectAttempts >= maxReconnectAttempts) {
        addLog('error', '최대 재연결 시도 횟수 초과')
      }
    }

  } catch (error) {
    addLog('error', `스트리밍 시작 실패: ${error.message}`)
  }
}

const stopStreaming = () => {
  if (eventSource) {
    eventSource.close()
    eventSource = null
  }
  
  if (reconnectTimeout) {
    clearTimeout(reconnectTimeout)
    reconnectTimeout = null
  }
  
  isConnected.value = false
  reconnectAttempts = 0
  addLog('info', '스트리밍 중지됨')
}

const handleMessage = (event) => {
  const message = {
    event: 'message',
    data: event.data,
    timestamp: new Date()
  }
  
  messages.value.unshift(message)
  
  // 메시지 수 제한 (최대 100개)
  if (messages.value.length > 100) {
    messages.value = messages.value.slice(0, 100)
  }
  
  updateLastUpdateTime()
}

const handleRobotStreamingImage = (event) => {
  const message = {
    event: 'robotStreamingImage',
    data: event.data,
    timestamp: new Date()
  }

  messages.value.unshift(message)
  if (messages.value.length > 100) {
    messages.value = messages.value.slice(0, 100)
  }

  try {
    // === 이미지 처리 추가 ===
    let imageUrl = null

    if (event.data.startsWith('image/jpg;base64,') || event.data.startsWith('image/jpeg;base64,')) {
      // Base64 → Blob 변환
      const base64Data = event.data.replace(/^image\/[^;]+;base64,/, '')
      const byteCharacters = atob(base64Data)
      const byteArray = Uint8Array.from(byteCharacters, c => c.charCodeAt(0))
      const blob = new Blob([byteArray], { type: 'image/jpeg' })
      imageUrl = URL.createObjectURL(blob)
    } else if (event.data.startsWith('http')) {
      // 이미지 URL 그대로 오는 경우
      imageUrl = event.data
    } else {
      // JSON 포맷 처리
      try {
        const data = JSON.parse(event.data)
        if (data.type === 'image' && data.image) {
          const byteCharacters = atob(data.image)
          const byteArray = Uint8Array.from(byteCharacters, c => c.charCodeAt(0))
          const blob = new Blob([byteArray], { type: 'image/jpeg' })
          imageUrl = URL.createObjectURL(blob)
        }
      } catch (e) {
        addLog('warning', '이미지 포맷이 아님')
      }
    }

    // 세팅
    if (imageUrl) {
      // 이전 URL 해제
      if (currentImage.value && currentImage.value.startsWith('blob:')) {
        URL.revokeObjectURL(currentImage.value)
      }
      currentImage.value = imageUrl
      imageTimestamp.value = new Date()
      addLog('success', '로봇 스트리밍 이미지 갱신 완료')
    }
    // ========================

  } catch (error) {
    addLog('error', `이미지 처리 실패: ${error.message}`)
  }

  updateLastUpdateTime()
}


const clearMessages = () => {
  messages.value = []
  
  // 이미지 URL 해제
  if (currentImage.value) {
    URL.revokeObjectURL(currentImage.value)
    currentImage.value = null
    imageTimestamp.value = null
  }
  
  addLog('info', '메시지 목록 및 이미지 지워짐')
}

const addLog = (type, message) => {
  const log = {
    type,
    message,
    timestamp: new Date()
  }
  
  connectionLogs.value.unshift(log)
  
  // 로그 수 제한 (최대 50개)
  if (connectionLogs.value.length > 50) {
    connectionLogs.value = connectionLogs.value.slice(0, 50)
  }
}

const updateLastUpdateTime = () => {
  lastUpdateTime.value = new Date().toLocaleTimeString('ko-KR')
}

const formatTime = (timestamp) => {
  if (!timestamp) return ''
  return timestamp.toLocaleTimeString('ko-KR', {
    hour: '2-digit',
    minute: '2-digit',
    second: '2-digit',
    fractionalSecondDigits: 3
  })
}

// Lifecycle
onMounted(() => {
  addLog('info', 'SSE 테스트 페이지 로드됨')
})

onUnmounted(() => {
  stopStreaming()
  
  // 이미지 URL 해제
  if (currentImage.value) {
    URL.revokeObjectURL(currentImage.value)
  }
})
</script>

<style scoped>
.sse-test-page {
  padding: 20px;
  height: 100%;
  background-color: #081028;
  color: #ffffff;
  display: flex;
  flex-direction: column;
  gap: 20px;
}

.header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding-bottom: 20px;
  border-bottom: 1px solid #2a2f3e;
}

.header h1 {
  margin: 0;
  font-size: 24px;
  font-weight: bold;
  color: #ffffff;
}

.controls {
  display: flex;
  gap: 12px;
}

.control-btn {
  text-transform: none;
}

.status-panel {
  display: flex;
  gap: 30px;
  padding: 16px;
  background-color: #1a1f2e;
  border-radius: 8px;
  border: 1px solid #2a2f3e;
}

.status-item {
  display: flex;
  align-items: center;
  gap: 8px;
}

.status-label {
  color: #8a92a6;
  font-size: 14px;
}

.status-value {
  color: #ffffff;
  font-weight: 500;
}

.content-area {
  display: grid;
  grid-template-columns: 2fr 1fr;
  gap: 20px;
  flex: 1;
}

.streaming-section, .settings-section {
  background-color: #1a1f2e;
  border-radius: 8px;
  padding: 20px;
  border: 1px solid #2a2f3e;
}

.streaming-section h3, .settings-section h3 {
  margin: 0 0 16px 0;
  font-size: 18px;
  font-weight: bold;
  color: #ffffff;
}

.streaming-container {
  height: 400px;
  overflow-y: auto;
  background-color: #222738;
  border-radius: 6px;
  padding: 12px;
}

.message-section {
  margin-bottom: 20px;
}

.section-title {
  margin: 0 0 12px 0;
  font-size: 16px;
  font-weight: bold;
  color: #3a57e8;
  padding: 8px 12px;
  background-color: rgba(58, 87, 232, 0.1);
  border-radius: 4px;
  border-left: 3px solid #3a57e8;
}

.message-section:last-child {
  margin-bottom: 0;
}

.image-display-section {
  margin-bottom: 20px;
  padding: 16px;
  background-color: #1a1f2e;
  border-radius: 8px;
  border: 2px solid #13c572;
}

.image-title {
  margin: 0 0 12px 0;
  font-size: 14px;
  font-weight: bold;
  color: #13c572;
}

.image-container {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 8px;
}

.streaming-image {
  max-width: 100%;
  max-height: 300px;
  border-radius: 6px;
  border: 1px solid #2a2f3e;
  box-shadow: 0 4px 8px rgba(0, 0, 0, 0.3);
}

.image-info {
  display: flex;
  justify-content: center;
  align-items: center;
  gap: 8px;
}

.image-timestamp {
  color: #8a92a6;
  font-size: 12px;
}

.message-item {
  margin-bottom: 12px;
  padding: 12px;
  background-color: #2a2f3e;
  border-radius: 6px;
  border-left: 4px solid #3a57e8;
}

.message-item.robotStreamingImage {
  border-left-color: #13c572;
}

.message-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 8px;
}

.event-type {
  background-color: #3a57e8;
  color: white;
  padding: 2px 8px;
  border-radius: 4px;
  font-size: 12px;
  font-weight: bold;
}

.message-item.robotStreamingImage .event-type {
  background-color: #13c572;
}

.timestamp {
  color: #8a92a6;
  font-size: 12px;
}

.message-content pre {
  margin: 0;
  color: #ffffff;
  font-size: 13px;
  white-space: pre-wrap;
  word-break: break-all;
}

.no-messages {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  height: 100%;
  color: #8a92a6;
  text-align: center;
}

.no-messages p {
  margin: 8px 0;
  font-size: 14px;
}

.settings-form {
  display: flex;
  flex-direction: column;
  gap: 16px;
}

.setting-input {
  width: 100%;
}

.setting-switch {
  margin-top: 8px;
}

.log-section {
  background-color: #1a1f2e;
  border-radius: 8px;
  padding: 20px;
  border: 1px solid #2a2f3e;
}

.log-section h3 {
  margin: 0 0 16px 0;
  font-size: 18px;
  font-weight: bold;
  color: #ffffff;
}

.log-container {
  height: 200px;
  overflow-y: auto;
  background-color: #222738;
  border-radius: 6px;
  padding: 12px;
}

.log-item {
  display: flex;
  gap: 12px;
  margin-bottom: 8px;
  padding: 8px;
  border-radius: 4px;
  font-size: 13px;
}

.log-item.info {
  background-color: rgba(58, 87, 232, 0.1);
  border-left: 3px solid #3a57e8;
}

.log-item.success {
  background-color: rgba(19, 197, 114, 0.1);
  border-left: 3px solid #13c572;
}

.log-item.warning {
  background-color: rgba(255, 215, 0, 0.1);
  border-left: 3px solid #ffd700;
}

.log-item.error {
  background-color: rgba(255, 71, 87, 0.1);
  border-left: 3px solid #ff4757;
}

.log-time {
  color: #8a92a6;
  font-size: 12px;
  min-width: 80px;
}

.log-message {
  color: #ffffff;
  flex: 1;
}

/* Vuetify 스타일 오버라이드 */
:deep(.v-text-field .v-field) {
  background-color: #2a2f3e;
  border-color: #3a3f4e;
}

:deep(.v-text-field .v-field__input) {
  color: #ffffff;
}

:deep(.v-text-field .v-field__label) {
  color: #8a92a6;
}

:deep(.v-switch .v-switch__track) {
  background-color: #3a3f4e;
}

:deep(.v-switch .v-switch__thumb) {
  background-color: #8a92a6;
}

:deep(.v-switch--density-default .v-switch--inset .v-switch__track) {
  background-color: #3a3f4e;
}

/* 반응형 디자인 */
@media (max-width: 768px) {
  .content-area {
    grid-template-columns: 1fr;
  }
  
  .header {
    flex-direction: column;
    gap: 16px;
    align-items: stretch;
  }
  
  .controls {
    justify-content: center;
  }
  
  .status-panel {
    flex-direction: column;
    gap: 12px;
  }
}
</style>