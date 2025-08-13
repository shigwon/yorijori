<template>
  <div class="dashboard-content">
      <!-- 카드 그리드 -->
      <div class="cards-container">
        <!-- 상단 카드들 -->
        <div class="top-cards">
          <!-- Robot Status 카드 -->
          <div class="card robot-status-card">
            <div class="card-header">
              <button class="card-title-btn robot-status-btn" @click="goToRobotStatus">Robot Status</button>
            </div>
            <div class="card-content">
              <div class="robot-container">
                <div class="robot-item">
                  <div class="robot-info">
                    <span class="robot-id">RB-001</span>
                    <span class="robot-location">Location: Zone A</span>
                    <span class="robot-order">Order: #5687</span>
                  </div>
                </div>
                <div class="robot-item">
                  <div class="robot-info">
                    <span class="robot-id">RB-002</span>
                    <span class="robot-location">Location: Zone B</span>
                    <span class="robot-order">Order: #4487</span>
                  </div>
                </div>
              </div>
            </div>
          </div>
  
          <!-- Log 카드 -->
          <div class="card log-card">
            <div class="card-header">
              <button class="card-title-btn log-btn" @click="goToLog">Log</button>
            </div>
            <div class="card-content">
              <div class="log-table">
                <div class="log-header">
                  <span class="log-col"></span>
                  <span class="log-col">order</span>
                  <span class="log-col">date</span>
                  <span class="log-col">status</span>
                </div>
                <div class="log-row" v-for="i in 5" :key="i">
                  <input type="checkbox" class="log-checkbox" />
                  <span class="log-order">&{{ i }}</span>
                  <span class="log-date">Dec 30, 10:24 AM</span>
                  <span class="log-status">End</span>
                </div>
              </div>
            </div>
          </div>
        </div>
  
        <!-- 중앙 카드들 -->
        <div class="middle-cards">
          <!-- Likes 카드 -->
          <div class="card likes-card">
            <div class="card-header">
              <button class="card-title-btn likes-btn" @click="goToLikes">Likes</button>
              <div class="dropdown">
                <span>This Week</span>
                <i class="dropdown-icon">▼</i>
              </div>
            </div>
            <div class="card-content">
              <div class="chart-container">
                <div class="donut-chart">
                  <div class="chart-outer"></div>
                  <div class="chart-inner"></div>
                </div>
                <div class="chart-legend">
                  <div class="legend-item">
                    <span class="legend-color blue"></span>
                    <span>Fashion 251K</span>
                  </div>
                  <div class="legend-item">
                    <span class="legend-color light-blue"></span>
                    <span>Accessories 176K</span>
                  </div>
                </div>
              </div>
            </div>
          </div>
  
          <!-- Chat 카드 -->
          <div class="card chat-card">
            <div class="card-header">
              <button class="card-title-btn chat-btn">Chat</button>
            </div>
            <div class="card-content">
              <div class="chat-grid">
                <div 
                  class="chat-button" 
                  v-for="i in 12" 
                  :key="i"
                  @click="openChatModal(`주문번호 ${i}번`)"
                >
                  주문번호 {{ i }}번
                </div>
              </div>
            </div>
          </div>
        </div>
  
        <!-- 하단 카드들 -->
        <div class="bottom-cards">
          <!-- Dashboard 카드 -->
          <div class="card dashboard-card">
            <div class="card-header">
              <button class="card-title-btn dashboard-btn" @click="goToDashboardDetail">Dashboard</button>
              <div class="dropdown">
                <span>This Week</span>
                <i class="dropdown-icon">▼</i>
              </div>
            </div>
            <div class="card-content">
              <div class="chart-container">
                <BarChart 
                  :data="chartData"
                  :categories="chartCategories"
                  :colors="chartColors"
                  :yAxisMax="160"
                  :yAxisStep="40"
                  height="200"
                />
              </div>
            </div>
          </div>
  
          <!-- Robot Position 카드 -->
          <div class="card position-card">
            <div class="card-header">
              <button class="card-title-btn position-btn" @click="goToRobotPosition">Robot Position</button>
            </div>
            <div class="card-content">
              <div class="map-container">
                <div class="map">
                  <div class="map-location">GS칼텍스</div>
                  <div class="map-location">홈플러스</div>
                  <div class="map-location">광산세무서</div>
                  <div class="map-location">수완중흥S클래스</div>
                  <div class="map-location">운남주공</div>
                  <div class="map-location">금구초등학교</div>
                  <div class="map-location">대반초등학교</div>
                  <div class="map-location">운남고등학교</div>
                  <div class="robot-marker"></div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
      
      <!-- Chat Modal -->
      <ChatModal 
        :isVisible="isModalVisible"
        :orderNumber="selectedOrderNumber"
        @close="closeChatModal"
      />
    </div>
  </template>
  
  <script setup>
  import { ref } from 'vue'
  import { useRouter } from 'vue-router'
  import BarChart from '../chart/BarChart.vue'
  import ChatModal from '../modal/ChatModal.vue'
  
  const router = useRouter()
  
  // Reactive data
  const chartData = ref([65, 85, 120, 95, 140, 110, 90])
  const chartCategories = ref(['M', 'T', 'W', 'T', 'F', 'S', 'S'])
  const chartColors = ref(['#1e40af'])
  const isModalVisible = ref(false)
  const selectedOrderNumber = ref('')
  
  // Methods
  const openChatModal = (orderNumber) => {
    selectedOrderNumber.value = orderNumber
    isModalVisible.value = true
  }
  
  const closeChatModal = () => {
    isModalVisible.value = false
    selectedOrderNumber.value = ''
  }
  
  const goToDashboardDetail = () => {
    // Dashboard 상세페이지로 이동
    router.push('/admin/dashboard-detail')
  }
  
  const goToLikes = () => {
    // Likes 페이지로 이동
    router.push('/admin/likes')
  }
  
  const goToLog = () => {
    // Log 페이지로 이동
    router.push('/admin/log')
  }
  
  const goToRobotStatus = () => {
    // Robot Status 페이지로 이동
    router.push('/admin/robot-status')
  }
  
  const goToRobotPosition = () => {
    // Robot Position 페이지로 이동
    router.push('/admin/robot-position')
  }
  </script>
  
  <style scoped>
  .dashboard-content {
    width: 100%;
    height: 100%;
    padding: 20px;
  }
  
  .cards-container {
    display: flex;
    flex-direction: column;
    gap: 20px;
  }
  
  .top-cards, .middle-cards, .bottom-cards {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 20px;
  }
  
  .card {
    background-color: #222738;
    border-radius: 12px;
    padding: 24px;
    box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
    min-height: 280px;
  }
  
  .card-header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-bottom: 20px;
  }
  
  .card-title-btn {
    background: none;
    border: none;
    color: #d3d3d3;
    font-size: 18px;
    font-weight: bold;
    cursor: pointer;
    padding: 8px 16px;
    border-radius: 8px;
    transition: all 0.3s ease;
    position: relative;
    overflow: hidden;
  }
  
  .card-title-btn::before {
    content: '';
    position: absolute;
    top: 0;
    left: -100%;
    width: 100%;
    height: 100%;
    background: linear-gradient(90deg, transparent, rgba(255, 255, 255, 0.1), transparent);
    transition: left 0.5s;
  }
  
  .card-title-btn:hover::before {
    left: 100%;
  }
  
  .card-title-btn:hover {
    transform: translateY(-2px);
    box-shadow: 0 4px 12px rgba(0, 0, 0, 0.3);
  }
  
  .card-title-btn:active {
    transform: translateY(0);
  }
  
  /* 각 카드별 버튼 색상 */
  .robot-status-btn {
    background: linear-gradient(135deg, #3a57e8, #5c6ca5);
    color: white;
  }
  
  .robot-status-btn:hover {
    background: linear-gradient(135deg, #4a67f8, #6c7cb5);
  }
  
  .log-btn {
    background: linear-gradient(135deg, #13c572, #10b981);
    color: white;
  }
  
  .log-btn:hover {
    background: linear-gradient(135deg, #14d582, #11ca91);
  }
  
  .likes-btn {
    background: linear-gradient(135deg, #ec4899, #f472b6);
    color: white;
  }
  
  .likes-btn:hover {
    background: linear-gradient(135deg, #f55aa9, #f582c6);
  }
  
  .chat-btn {
    background: linear-gradient(135deg, #8b5cf6, #a78bfa);
    color: white;
  }
  
  .chat-btn:hover {
    background: linear-gradient(135deg, #9b6c06, #b78bfa);
  }
  
  .dashboard-btn {
    background: linear-gradient(135deg, #06b6d4, #22d3ee);
    color: white;
  }
  
  .dashboard-btn:hover {
    background: linear-gradient(135deg, #16c6e4, #32e3fe);
  }
  
  .position-btn {
    background: linear-gradient(135deg, #f59e0b, #fbbf24);
    color: white;
  }
  
  .position-btn:hover {
    background: linear-gradient(135deg, #05ae1b, #fccf34);
  }
  
  .dropdown {
    display: flex;
    align-items: center;
    gap: 8px;
    color: #8a92a6;
    cursor: pointer;
  }
  
  .dropdown-icon {
    font-size: 12px;
  }
  
  .robot-item {
    padding: 12px;
    background-color: #2a2f3e;
    border-radius: 8px;
    margin-bottom: 12px;
  }
  
  .robot-item:hover {
    background-color: #3a57e8;;
  }
  
  .robot-info {
    display: flex;
    flex-direction: column;
    gap: 4px;
  }
  
  .robot-id {
    font-weight: bold;
    color: #b6bace;
  }
  
  .robot-location, .robot-order {
    font-size: 14px;
    color: #8a92a6;
  }
  
  .log-table {
    width: 100%;
  }
  
  .log-header {
    display: grid;
    grid-template-columns: 30px 1fr 1fr 1fr;
    gap: 12px;
    padding: 8px 0;
    border-bottom: 1px solid #2a2f3e;
    font-weight: bold;
    font-size: 14px;
  }
  
  .log-row {
    display: grid;
    grid-template-columns: 30px 1fr 1fr 1fr;
    gap: 12px;
    padding: 8px 0;
    border-radius: 6px;
    align-items: center;
    font-size: 14px;
  }
  
  .log-row:hover {
    background-color: #3a57e8;
  }
  
  .log-checkbox {
    width: 16px;
    height: 16px;
  }
  
  .log-status {
    background-color: #13c572;
    color: white;
    padding: 4px 8px;
    border-radius: 4px;
    font-size: 12px;
    width: fit-content;
  }
  
  .chat-grid {
    display: grid;
    grid-template-columns: repeat(4, 1fr);
    gap: 28px;
    padding-top: 20px;
  }
  
  .chat-button {
    background-color: #2a2f3e;
    padding: 12px 8px;
    border-radius: 6px;
    text-align: center;
    font-size: 12px;
    cursor: pointer;
    transition: background-color 0.3s;
  }
  
  .chat-button:hover {
    background-color: #3a57e8;
  }
  
  .chart-container {
    display: flex;
    align-items: center;
    gap: 30px;
    padding: 20px 0;
  }
  
  .donut-chart {
    position: relative;
    width: 180px;
    height: 180px;
  }
  
  .chart-outer {
    width: 180px;
    height: 180px;
    border-radius: 50%;
    border: 12px solid #3a57e8;
    border-top-color: transparent;
    transform: rotate(-45deg);
  }
  
  .chart-inner {
    position: absolute;
    top: 30px;
    left: 30px;
    width: 120px;
    height: 120px;
    border-radius: 50%;
    border: 10px solid #8a92a6;
    border-top-color: transparent;
    transform: rotate(-45deg);
  }
  
  .chart-legend {
    display: flex;
    flex-direction: column;
    gap: 25px;
    padding-left: 40px;
  }
  
  .legend-item {
    display: flex;
    align-items: center;
    gap: 12px;
    font-size: 16px;
    font-weight: 500;
  }
  
  .legend-color {
    width: 16px;
    height: 16px;
    border-radius: 50%;
  }
  
  .legend-color.blue {
    background-color: #3a57e8;
  }
  
  .legend-color.light-blue {
    background-color: #5c6ca5;
  }
  
  /* BarChart 컴포넌트 스타일링 */
  .chart-container {
    height: 200px;
    width: 100%;
  }
  
  .map-container {
    position: relative;
    height: 200px;
    background-color: #2a2f3e;
    border-radius: 8px;
    padding: 16px;
  }
  
  .map {
    position: relative;
    height: 100%;
    background-color: #1a1f2e;
    border-radius: 6px;
  }
  
  .map-location {
    position: absolute;
    font-size: 12px;
    color: #8a92a6;
    padding: 4px 8px;
    background-color: #222738;
    border-radius: 4px;
  }
  
  .robot-container {
    display: flex;
    flex-direction: column;
    gap: 16px;
    padding-top: 12px;
  }
  
  .map-location:nth-child(1) { top: 10%; left: 20%; }
  .map-location:nth-child(2) { top: 20%; left: 60%; }
  .map-location:nth-child(3) { top: 40%; left: 10%; }
  .map-location:nth-child(4) { top: 50%; left: 70%; }
  .map-location:nth-child(5) { top: 60%; left: 30%; }
  .map-location:nth-child(6) { top: 70%; left: 80%; }
  .map-location:nth-child(7) { top: 80%; left: 15%; }
  .map-location:nth-child(8) { top: 90%; left: 50%; }
  
  .robot-marker {
    position: absolute;
    top: 50%;
    left: 50%;
    width: 12px;
    height: 12px;
    background-color: #ff4757;
    border-radius: 50%;
    transform: translate(-50%, -50%);
  }
  
  /* 반응형 디자인 */
  @media (max-width: 1200px) {
    .top-cards, .middle-cards, .bottom-cards {
      grid-template-columns: 1fr;
    }
    
    .card-title-btn {
      font-size: 16px;
      padding: 6px 12px;
    }
  }
  
  @media (max-width: 768px) {
    .top-cards, .middle-cards, .bottom-cards {
      grid-template-columns: 1fr;
    }
    
    .card-title-btn {
      font-size: 14px;
      padding: 4px 8px;
    }
    
    .card-header {
      flex-direction: column;
      gap: 8px;
      align-items: flex-start;
    }
  }
  
  @media (max-width: 480px) {
    .card-title-btn {
      font-size: 12px;
      padding: 3px 6px;
    }
    
    .card {
      padding: 16px;
      min-height: 240px;
    }
  }
  </style>