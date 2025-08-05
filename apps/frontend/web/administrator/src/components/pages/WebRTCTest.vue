<template>
    <div class="websocket-test">
      <h2>WebRTC 연결 테스트 (OpenVidu)</h2>
      
      <!-- 서버 설정 섹션 -->
      <div class="api-section">
        <h3>1. 서버 설정</h3>
        
        <div class="input-group">
          <label for="robotId">robot ID:</label>
          <input 
            id="robotId" 
            v-model="robotId" 
            type="text" 
            placeholder="1"
          />
        </div>
        
        <div class="input-group">
          <label for="role">Role:</label>
          <select id="role" v-model="role">
            <option value="SUBSCRIBER">SUBSCRIBER</option>
            <option value="PUBLISHER">PUBLISHER</option>
          </select>
        </div>
      </div>
  
      <!-- 토큰 발급 섹션 -->
      <div class="websocket-section">
        <h3>2. WebRTC 토큰 발급</h3>
        
        <button @click="getToken" :disabled="isLoading">
          {{ isLoading ? '토큰 발급 중...' : '토큰 발급' }}
        </button>
        
        <div v-if="tokenResponse" class="response-section">
          <h4>토큰 응답:</h4>
          <pre>{{ JSON.stringify(tokenResponse, null, 2) }}</pre>
        </div>
        
        <div v-if="token" class="token-section">
          <h4>발급된 토큰:</h4>
          <div class="token-display">
            <code>{{ token }}</code>
            <button @click="copyToken" class="copy-btn">복사</button>
          </div>
        </div>
      </div>
  
      <!-- WebRTC 연결 테스트 -->
      <div class="auto-test-section">
        <h3>3. WebRTC 연결 테스트</h3>
        
        <div class="connection-status">
          <h4>연결 상태:</h4>
          <div class="status-indicator" :class="{ connected: isConnected, disconnected: !isConnected }">
            {{ isConnected ? '연결됨' : '연결 안됨' }}
          </div>
        </div>
        
        <div class="button-group">
          <button @click="connectWebRTC" :disabled="!token || isConnected">
            WebRTC 연결
          </button>
          <button @click="disconnectWebRTC" :disabled="!isConnected">
            연결 해제
          </button>
        </div>
        
        <div v-if="messages.length > 0" class="messages-section">
          <h4>연결 로그:</h4>
          <div class="messages-container">
            <div 
              v-for="(message, index) in messages" 
              :key="index" 
              class="message"
              :class="message.type"
            >
              <span class="timestamp">{{ message.timestamp }}</span>
              <span class="content">{{ message.content }}</span>
            </div>
          </div>
          <button @click="clearMessages" class="clear-btn">로그 지우기</button>
        </div>
      </div>
  
      <!-- 자동 테스트 -->
      <div class="auto-test-section">
        <h3>4. 자동 테스트</h3>
        <button @click="runAutoTest" :disabled="isAutoTesting">
          {{ isAutoTesting ? '테스트 중...' : '토큰 발급 후 자동 연결' }}
        </button>
      </div>
    </div>
  </template>
  
  <script setup>
  import { ref, onUnmounted } from 'vue'
  import axios from 'axios'
  
  // Reactive data
  const robotId = ref('1')
  const role = ref('SUBSCRIBER')
  const isLoading = ref(false)
  const tokenResponse = ref(null)
  const token = ref('')
  const isConnected = ref(false)
  const messages = ref([])
  const isAutoTesting = ref(false)
  
  // Methods
  // 토큰 발급 메서드
  const getToken = async () => {
    isLoading.value = true;
    clearMessages();
    
    try {
      addMessage('info', '토큰 발급 요청 중...');
      
      // 반드시 프록시를 타도록 상대경로로 요청
      const response = await axios.post('/api/v1/stream/join', {
        robotId: robotId.value,
        role: role.value
      });
  
      const { data } = response;
      tokenResponse.value = data;
      
      if (data.result === 'success' && data.data) {
        token.value = data.data;
        addMessage('success', `토큰 발급 성공: ${data.msg}`);
      } else {
        addMessage('error', `토큰 발급 실패: ${data.msg || '알 수 없는 오류'}`);
      }
    } catch (error) {
      console.error('토큰 발급 오류:', error);
      
      if (error.response) {
        const errorMessage = error.response.data?.message || '토큰 발급에 실패했습니다.';
        addMessage('error', errorMessage);
      } else if (error.request) {
        addMessage('error', '서버에 연결할 수 없습니다. 네트워크 연결을 확인해주세요.');
      } else {
        addMessage('error', '토큰 발급 중 오류가 발생했습니다.');
      }
    } finally {
      isLoading.value = false;
    }
  }
  
  // WebRTC 연결 메서드
  const connectWebRTC = () => {
    if (!token.value) {
      addMessage('error', '토큰이 없습니다. 먼저 토큰을 발급해주세요.');
      return;
    }
  
    try {
      addMessage('info', 'WebRTC 연결 시도 중...');
      
      // OpenVidu WebRTC 연결 시뮬레이션
      // 실제 구현에서는 OpenVidu JavaScript SDK를 사용
      setTimeout(() => {
        isConnected.value = true;
        addMessage('success', 'WebRTC 연결 성공!');
        addMessage('info', `robot ID: ${robotId.value}`);
        addMessage('info', `Role: ${role.value}`);
      }, 2000);
      
    } catch (error) {
      addMessage('error', `WebRTC 연결 실패: ${error.message}`);
    }
  }
  
  // WebRTC 연결 해제
  const disconnectWebRTC = () => {
    isConnected.value = false;
    addMessage('info', 'WebRTC 연결 해제됨');
  }
  
  // 토큰 복사
  const copyToken = () => {
    if (token.value) {
      navigator.clipboard.writeText(token.value).then(() => {
        addMessage('success', '토큰이 클립보드에 복사되었습니다.');
      }).catch(() => {
        addMessage('error', '토큰 복사에 실패했습니다.');
      });
    }
  }
  
  // 메시지 추가
  const addMessage = (type, content) => {
    const timestamp = new Date().toLocaleTimeString();
    messages.value.unshift({
      type,
      content,
      timestamp
    });
  }
  
  // 메시지 로그 지우기
  const clearMessages = () => {
    messages.value = [];
  }
  
  // 자동 테스트
  const runAutoTest = async () => {
    isAutoTesting.value = true;
    clearMessages();
    
    try {
      // 1. 토큰 발급
      addMessage('info', '자동 테스트 시작...');
      await getToken();
      
      // 2. 잠시 대기 후 WebRTC 연결
      setTimeout(() => {
        if (token.value) {
          addMessage('info', 'WebRTC 연결 시도...');
          connectWebRTC();
        }
      }, 1000);
      
    } catch (error) {
      addMessage('error', `자동 테스트 실패: ${error.message}`);
    } finally {
      isAutoTesting.value = false;
    }
  }
  
  // 컴포넌트 언마운트 시 연결 해제
  onUnmounted(() => {
    disconnectWebRTC();
  })
  </script>
  
  <style scoped>
  .websocket-test {
    max-width: 800px;
    margin: 0 auto;
    padding: 20px;
    font-family: Arial, sans-serif;
  }
  
  h2 {
    color: #333;
    text-align: center;
    margin-bottom: 30px;
  }
  
  h3 {
    color: #555;
    margin-bottom: 15px;
    padding-bottom: 5px;
    border-bottom: 2px solid #eee;
  }
  
  .api-section, .websocket-section, .auto-test-section {
    background: #f9f9f9;
    padding: 20px;
    margin-bottom: 20px;
    border-radius: 8px;
    border: 1px solid #ddd;
  }
  
  .input-group {
    margin-bottom: 15px;
  }
  
  .input-group label {
    display: block;
    margin-bottom: 5px;
    font-weight: bold;
    color: #555;
  }
  
  .input-group input, .input-group select {
    width: 100%;
    padding: 8px;
    border: 1px solid #ddd;
    border-radius: 4px;
    font-size: 14px;
  }
  
  button {
    background: #007bff;
    color: white;
    border: none;
    padding: 10px 20px;
    border-radius: 4px;
    cursor: pointer;
    font-size: 14px;
    margin-right: 10px;
  }
  
  button:hover:not(:disabled) {
    background: #0056b3;
  }
  
  button:disabled {
    background: #ccc;
    cursor: not-allowed;
  }
  
  .button-group {
    margin-bottom: 15px;
  }
  
  .response-section {
    margin-top: 15px;
    padding: 15px;
    background: #f8f9fa;
    border-radius: 4px;
    border: 1px solid #dee2e6;
  }
  
  .response-section pre {
    background: #fff;
    padding: 10px;
    border-radius: 4px;
    overflow-x: auto;
    font-size: 12px;
  }
  
  .token-section {
    margin-top: 15px;
    padding: 15px;
    background: #e8f5e8;
    border-radius: 4px;
    border: 1px solid #28a745;
  }
  
  .token-display {
    display: flex;
    align-items: center;
    gap: 10px;
    margin-top: 10px;
  }
  
  .token-display code {
    flex: 1;
    background: #fff;
    padding: 10px;
    border-radius: 4px;
    font-size: 12px;
    word-break: break-all;
    border: 1px solid #ddd;
  }
  
  .copy-btn {
    background: #28a745;
    font-size: 12px;
    padding: 8px 12px;
    white-space: nowrap;
  }
  
  .copy-btn:hover {
    background: #218838;
  }
  
  .connection-status {
    margin-bottom: 15px;
  }
  
  .status-indicator {
    display: inline-block;
    padding: 8px 16px;
    border-radius: 20px;
    font-weight: bold;
    color: white;
  }
  
  .status-indicator.connected {
    background: #28a745;
  }
  
  .status-indicator.disconnected {
    background: #dc3545;
  }
  
  .messages-section {
    margin-top: 15px;
  }
  
  .messages-container {
    max-height: 300px;
    overflow-y: auto;
    border: 1px solid #ddd;
    border-radius: 4px;
    background: white;
    margin-bottom: 10px;
  }
  
  .message {
    padding: 8px 12px;
    border-bottom: 1px solid #eee;
    font-size: 12px;
  }
  
  .message:last-child {
    border-bottom: none;
  }
  
  .message.success {
    background: #d4edda;
    color: #155724;
  }
  
  .message.error {
    background: #f8d7da;
    color: #721c24;
  }
  
  .message.info {
    background: #d1ecf1;
    color: #0c5460;
  }
  
  .message.received {
    background: #fff3cd;
    color: #856404;
  }
  
  .timestamp {
    font-weight: bold;
    margin-right: 10px;
  }
  
  .clear-btn {
    background: #6c757d;
    font-size: 12px;
    padding: 5px 10px;
  }
  
  .clear-btn:hover {
    background: #545b62;
  }
  </style> 