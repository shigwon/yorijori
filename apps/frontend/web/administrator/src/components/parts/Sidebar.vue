<template>
    <div class="sidebar">
      <div class="logo">
        <!-- <img src="./linky2-1-1.png" alt="Linky" class="logo-img" /> -->
        <span class="logo-text">Linky</span>
      </div>
      
      <nav class="nav-menu">
        <div 
          class="nav-item" 
          
          @click="setActiveMenu('dashboard')"
        >
          <i class="nav-icon">📊</i>
          <span>Dashboard</span>
        </div>
        <div 
          class="nav-item" 
          :class="{ active: activeMenu === 'robot-status' }"
          @click="setActiveMenu('robot-status')"
        >
          <i class="nav-icon">🤖</i>
          <span>Robot Status</span>
        </div>
        <div 
          class="nav-item" 
          :class="{ active: activeMenu === 'likes' }"
          @click="setActiveMenu('likes')"
        >
          <i class="nav-icon">❤️</i>
          <span>Likes</span>
        </div>
        <div 
          class="nav-item" 
          :class="{ active: activeMenu === 'chat' }"
          @click="setActiveMenu('chat')"
        >
          <i class="nav-icon">💬</i>
          <span>Chat</span>
        </div>
        <div 
          class="nav-item" 
          :class="{ active: activeMenu === 'calendar' }"
          @click="setActiveMenu('calendar')"
        >
          <i class="nav-icon">📅</i>
          <span>Calendar</span>
        </div>
        <div 
          class="nav-item" 
          :class="{ active: activeMenu === 'system-logs' }"
          @click="setActiveMenu('system-logs')"
        >
          <i class="nav-icon">💻</i>
          <span>System Logs</span>
        </div>
        <div 
          class="nav-item" 
          :class="{ active: activeMenu === 'driving-stream' }"
          @click="setActiveMenu('driving-stream')"
        >
          <i class="nav-icon">🎥</i>
          <span>Driving Stream</span>
        </div>
        <div 
          class="nav-item" 
          :class="{ active: activeMenu === 'robot-position' }"
          @click="setActiveMenu('robot-position')"
        >
          <i class="nav-icon">📍</i>
          <span>Robot Position</span>
        </div>
      </nav>
      
      <div class="sidebar-bottom">
        <div class="nav-item">
          <i class="nav-icon">⚙️</i>
          <span>Settings</span>
        </div>
        <div class="nav-item">
          <i class="nav-icon">⏻</i>
          <span>Power</span>
        </div>
      </div>
    </div>
  </template>
  
  <script setup>
  import { ref } from 'vue'
  import { useRouter } from 'vue-router'
  import { defineEmits } from 'vue'
  
  const router = useRouter()
  const emit = defineEmits(['menu-change'])
  

// 현재 활성 메뉴
const activeMenu = ref('dashboard')

// 라우트 매핑
const routes = {
  'dashboard': '/dashboard-detail',
  'robot-status': '/robot-status',
  'likes': '/likes',
  'chat': '/chat',
  'calendar': '/calendar',
  'system-logs': '/system-logs',
  'driving-stream': '/driving-stream',
  'robot-position': '/robot-position'
}

// 메뉴 클릭 핸들러
const setActiveMenu = (menu) => {
  activeMenu.value = menu
  emit('menu-change', menu)
  
  // 라우트 이동
  if (routes[menu]) {
    router.push(routes[menu])
  }
}

// 필요한 경우 개별 메뉴 함수들
const goToDashboard = () => setActiveMenu('dashboard')
const goToRobotStatus = () => setActiveMenu('robot-status')
const goToLikes = () => setActiveMenu('likes')
const goToChat = () => setActiveMenu('chat')
const goToCalendar = () => setActiveMenu('calendar')
const goToSystemLogs = () => setActiveMenu('system-logs')
const goToDrivingStream = () => setActiveMenu('driving-stream')
const goToRobotPosition = () => setActiveMenu('robot-position')

  </script>
  
  <style scoped>
  .sidebar {
    width: 250px;
    background-color: #1a1f2e;
    padding: 20px;
    display: flex;
    flex-direction: column;
  }
  
  .logo {
    display: flex;
    align-items: center;
    margin-bottom: 40px;
  }
  
  .logo-img {
    width: 40px;
    height: 40px;
    margin-right: 10px;
  }
  
  .logo-text {
    font-size: 24px;
    font-weight: bold;
    color: #3dade5;
  }
  
  .nav-menu {
    flex: 1;
  }
  
  .nav-item {
    display: flex;
    align-items: center;
    padding: 12px 16px;
    margin-bottom: 8px;
    border-radius: 8px;
    cursor: pointer;
    transition: all 0.3s ease;
  }
  
  .nav-item:hover {
    background-color: #4774b6;
    transform: translateX(5px);
  }
  
  .nav-item.active {
    background-color: #3a57e8;
    box-shadow: 0 2px 8px rgba(58, 87, 232, 0.3);
  }
  
  .nav-icon {
    margin-right: 12px;
    font-size: 18px;
  }
  
  .sidebar-bottom {
    margin-top: auto;
  }
  
  /* 반응형 디자인 */
  @media (max-width: 768px) {
    .sidebar {
      width: 200px;
      padding: 15px;
    }
    
    .nav-item {
      padding: 10px 12px;
      font-size: 14px;
    }
    
    .nav-icon {
      font-size: 16px;
    }
  }
  
  @media (max-width: 480px) {
    .sidebar {
      width: 100%;
      height: auto;
      padding: 10px;
    }
    
    .nav-menu {
      display: flex;
      flex-wrap: wrap;
      gap: 8px;
    }
    
    .nav-item {
      flex: 1;
      min-width: 120px;
      margin-bottom: 0;
    }
  }
  </style> 