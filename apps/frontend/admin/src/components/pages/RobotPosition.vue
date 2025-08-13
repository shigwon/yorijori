<template>
    <div class="robot-position-page">
      <!-- 헤더 -->
      <div class="page-header">
        <h1 class="page-title">Robot Position</h1>
      </div>
  
      <!-- 지도 컨테이너 -->
      <div class="map-container">
        <div class="map">
          <!-- 지도 배경 -->
          <div class="map-background">
            <!-- 도로 네트워크 -->
            <div class="road-network">
              <div class="road horizontal-road"></div>
              <div class="road vertical-road"></div>
              <div class="road diagonal-road"></div>
            </div>
            
            <!-- 건물들 -->
            <div class="buildings">
              <!-- 주유소들 -->
              <div class="building gas-station" style="top: 15%; left: 20%;">
                <div class="building-icon">⛽</div>
                <div class="building-label">SK엔크린</div>
              </div>
              <div class="building gas-station" style="top: 25%; left: 75%;">
                <div class="building-icon">⛽</div>
                <div class="building-label">GS칼텍스</div>
              </div>
              
              <!-- 세무서 -->
              <div class="building government" style="top: 35%; left: 15%;">
                <div class="building-icon">🏛️</div>
                <div class="building-label">광산세무서</div>
              </div>
              
              <!-- 대형마트 -->
              <div class="building supermarket" style="top: 45%; left: 70%;">
                <div class="building-icon">🏪</div>
                <div class="building-label">홈플러스</div>
              </div>
              
              <!-- 아파트 단지들 -->
              <div class="building apartment" style="top: 20%; left: 40%;">
                <div class="building-icon">🏢</div>
                <div class="building-label">주공4단지</div>
              </div>
              <div class="building apartment" style="top: 30%; left: 60%;">
                <div class="building-icon">🏢</div>
                <div class="building-label">수완중흥S클래스</div>
              </div>
              <div class="building apartment" style="top: 50%; left: 25%;">
                <div class="building-icon">🏢</div>
                <div class="building-label">휴먼시아3단지</div>
              </div>
              <div class="building apartment" style="top: 60%; left: 45%;">
                <div class="building-icon">🏢</div>
                <div class="building-label">운남주공8단지</div>
              </div>
              <div class="building apartment" style="top: 70%; left: 35%;">
                <div class="building-icon">🏢</div>
                <div class="building-label">주공9단지</div>
              </div>
              <div class="building apartment" style="top: 80%; left: 55%;">
                <div class="building-icon">🏢</div>
                <div class="building-label">운남주공5단지</div>
              </div>
              <div class="building apartment" style="top: 25%; left: 85%;">
                <div class="building-icon">🏢</div>
                <div class="building-label">하남광주부영</div>
              </div>
              <div class="building apartment" style="top: 40%; left: 90%;">
                <div class="building-icon">🏢</div>
                <div class="building-label">월곡2종공10단지</div>
              </div>
              <div class="building apartment" style="top: 55%; left: 80%;">
                <div class="building-icon">🏢</div>
                <div class="building-label">운남아이유쉘</div>
              </div>
              <div class="building apartment" style="top: 75%; left: 70%;">
                <div class="building-icon">🏢</div>
                <div class="building-label">운남주공7단지</div>
              </div>
              
              <!-- 학교들 -->
              <div class="building school" style="top: 65%; left: 20%;">
                <div class="building-icon">🏫</div>
                <div class="building-label">금구초등학교</div>
              </div>
              <div class="building school" style="top: 85%; left: 40%;">
                <div class="building-icon">🏫</div>
                <div class="building-label">대반초등학교</div>
              </div>
              <div class="building school" style="top: 90%; left: 60%;">
                <div class="building-icon">🏫</div>
                <div class="building-label">운남고등학교</div>
              </div>
              
              <!-- 하천 -->
              <div class="building river" style="top: 35%; left: 50%;">
                <div class="building-icon">🌊</div>
                <div class="building-label">풍영정천</div>
              </div>
            </div>
            
            <!-- 로봇 위치 표시 -->
            <div class="robot-location">
              <div class="robot-icon">🤖</div>
              <div class="robot-label">Robot-001</div>
            </div>
            
            <!-- 사람 위치 표시 -->
            <div class="person-location">
              <div class="person-icon">👤</div>
              <div class="person-label">User</div>
            </div>
            
            <!-- 연결선 -->
            <div class="connection-line"></div>
            
            <!-- 빨간 점들 (관심 지점) -->
            <div class="red-dot" style="top: 22%; left: 42%;"></div>
            <div class="red-dot" style="top: 32%; left: 62%;"></div>
            <div class="red-dot" style="top: 52%; left: 27%;"></div>
            <div class="red-dot" style="top: 62%; left: 47%;"></div>
            <div class="red-dot" style="top: 72%; left: 37%;"></div>
            <div class="red-dot" style="top: 82%; left: 57%;"></div>
            <div class="red-dot" style="top: 27%; left: 87%;"></div>
            <div class="red-dot" style="top: 42%; left: 92%;"></div>
            <div class="red-dot" style="top: 57%; left: 82%;"></div>
            <div class="red-dot" style="top: 77%; left: 72%;"></div>
          </div>
        </div>
      </div>
  
      <!-- 컨트롤 패널 -->
      <div class="control-panel">
        <div class="panel-section">
          <h3 class="section-title">Robot Status</h3>
          <div class="status-item">
            <span class="status-label">Robot-001:</span>
            <span class="status-value online">Online</span>
          </div>
          <div class="status-item">
            <span class="status-label">Battery:</span>
            <span class="status-value">85%</span>
          </div>
          <div class="status-item">
            <span class="status-label">Speed:</span>
            <span class="status-value">15 km/h</span>
          </div>
        </div>
        
        <div class="panel-section">
          <h3 class="section-title">Controls</h3>
          <button class="control-btn" @click="centerMap">Center Map</button>
          <button class="control-btn" @click="refreshLocation">Refresh Location</button>
          <button class="control-btn" @click="toggleTracking">Toggle Tracking</button>
        </div>
      </div>
    </div>
  </template>
  
  <script setup>
  import { ref } from 'vue'
  
  // Reactive data
  const isTracking = ref(true)
  
  // Methods
  const centerMap = () => {
    console.log('Centering map...')
    // 지도 중앙화 로직
  }
  
  const refreshLocation = () => {
    console.log('Refreshing robot location...')
    // 로봇 위치 새로고침 로직
  }
  
  const toggleTracking = () => {
    isTracking.value = !isTracking.value
    console.log('Tracking:', isTracking.value ? 'ON' : 'OFF')
    // 추적 토글 로직
  }
  </script>
  
  <style scoped>
  .robot-position-page {
    padding: 20px;
    height: 100%;
    background-color: #081028;
    color: #ffffff;
    display: flex;
    flex-direction: column;
  }
  
  /* 헤더 */
  .page-header {
    margin-bottom: 20px;
  }
  
  .page-title {
    font-size: 24px;
    font-weight: bold;
    margin: 0;
    color: #ffffff;
  }
  
  /* 지도 컨테이너 */
  .map-container {
    flex: 1;
    background-color: #1a1f2e;
    border-radius: 12px;
    overflow: hidden;
    margin-bottom: 20px;
  }
  
  .map {
    width: 100%;
    height: 100%;
    min-height: 500px;
    position: relative;
    background-color: #0f141f;
  }
  
  .map-background {
    width: 100%;
    height: 100%;
    position: relative;
    background: linear-gradient(135deg, #1a1f2e 0%, #0f141f 100%);
  }
  
  /* 도로 네트워크 */
  .road-network {
    position: absolute;
    width: 100%;
    height: 100%;
  }
  
  .road {
    position: absolute;
    background-color: #374151;
    border-radius: 2px;
  }
  
  .horizontal-road {
    top: 50%;
    left: 0;
    width: 100%;
    height: 4px;
    transform: translateY(-50%);
  }
  
  .vertical-road {
    top: 0;
    left: 50%;
    width: 4px;
    height: 100%;
    transform: translateX(-50%);
  }
  
  .diagonal-road {
    top: 20%;
    left: 20%;
    width: 60%;
    height: 4px;
    transform: rotate(45deg);
  }
  
  /* 건물들 */
  .buildings {
    position: absolute;
    width: 100%;
    height: 100%;
  }
  
  .building {
    position: absolute;
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 4px;
    cursor: pointer;
    transition: transform 0.3s ease;
  }
  
  .building:hover {
    transform: scale(1.1);
  }
  
  .building-icon {
    font-size: 20px;
    filter: drop-shadow(0 2px 4px rgba(0, 0, 0, 0.3));
  }
  
  .building-label {
    font-size: 10px;
    color: #b6bace;
    text-align: center;
    max-width: 80px;
    line-height: 1.2;
    background-color: rgba(0, 0, 0, 0.7);
    padding: 2px 4px;
    border-radius: 4px;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }
  
  /* 건물 타입별 색상 */
  .gas-station .building-icon {
    color: #f59e0b;
  }
  
  .government .building-icon {
    color: #3a57e8;
  }
  
  .supermarket .building-icon {
    color: #13c572;
  }
  
  .apartment .building-icon {
    color: #8b5cf6;
  }
  
  .school .building-icon {
    color: #ec4899;
  }
  
  .river .building-icon {
    color: #06b6d4;
  }
  
  /* 로봇 위치 */
  .robot-location {
    position: absolute;
    top: 30%;
    left: 50%;
    transform: translate(-50%, -50%);
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 4px;
    z-index: 10;
  }
  
  .robot-icon {
    font-size: 24px;
    color: #3a57e8;
    filter: drop-shadow(0 2px 4px rgba(0, 0, 0, 0.5));
  }
  
  .robot-label {
    font-size: 12px;
    color: #3a57e8;
    font-weight: bold;
    background-color: rgba(58, 87, 232, 0.2);
    padding: 2px 6px;
    border-radius: 4px;
  }
  
  /* 사람 위치 */
  .person-location {
    position: absolute;
    top: 70%;
    left: 50%;
    transform: translate(-50%, -50%);
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 4px;
    z-index: 10;
  }
  
  .person-icon {
    font-size: 20px;
    color: #13c572;
    filter: drop-shadow(0 2px 4px rgba(0, 0, 0, 0.5));
  }
  
  .person-label {
    font-size: 12px;
    color: #13c572;
    font-weight: bold;
    background-color: rgba(19, 197, 114, 0.2);
    padding: 2px 6px;
    border-radius: 4px;
  }
  
  /* 연결선 */
  .connection-line {
    position: absolute;
    top: 30%;
    left: 50%;
    width: 2px;
    height: 40%;
    background: linear-gradient(to bottom, #3a57e8, #13c572);
    transform: translateX(-50%);
    z-index: 5;
  }
  
  /* 빨간 점들 */
  .red-dot {
    position: absolute;
    width: 6px;
    height: 6px;
    background-color: #ff4757;
    border-radius: 50%;
    box-shadow: 0 0 4px rgba(255, 71, 87, 0.6);
    animation: pulse 2s infinite;
  }
  
  @keyframes pulse {
    0% {
      transform: scale(1);
      opacity: 1;
    }
    50% {
      transform: scale(1.5);
      opacity: 0.7;
    }
    100% {
      transform: scale(1);
      opacity: 1;
    }
  }
  
  /* 컨트롤 패널 */
  .control-panel {
    display: flex;
    gap: 20px;
    background-color: #222738;
    border-radius: 12px;
    padding: 20px;
  }
  
  .panel-section {
    flex: 1;
  }
  
  .section-title {
    font-size: 16px;
    font-weight: bold;
    margin: 0 0 12px 0;
    color: #ffffff;
  }
  
  .status-item {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 8px 0;
    border-bottom: 1px solid #2a2f3e;
  }
  
  .status-label {
    font-size: 14px;
    color: #b6bace;
  }
  
  .status-value {
    font-size: 14px;
    font-weight: 500;
  }
  
  .status-value.online {
    color: #13c572;
  }
  
  .control-btn {
    display: block;
    width: 100%;
    padding: 8px 12px;
    margin-bottom: 8px;
    background-color: #374151;
    border: none;
    border-radius: 6px;
    color: #ffffff;
    font-size: 12px;
    cursor: pointer;
    transition: background-color 0.3s ease;
  }
  
  .control-btn:hover {
    background-color: #3a57e8;
  }
  
  /* 반응형 디자인 */
  @media (max-width: 768px) {
    .control-panel {
      flex-direction: column;
    }
    
    .map {
      min-height: 300px;
    }
    
    .building-label {
      font-size: 8px;
      max-width: 60px;
    }
    
    .robot-icon {
      font-size: 20px;
    }
    
    .person-icon {
      font-size: 18px;
    }
  }
  
  @media (max-width: 480px) {
    .robot-position-page {
      padding: 10px;
    }
    
    .page-title {
      font-size: 20px;
    }
    
    .control-panel {
      padding: 15px;
    }
    
    .map {
      min-height: 250px;
    }
  }
  </style>