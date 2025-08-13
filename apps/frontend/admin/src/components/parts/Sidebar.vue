<template>
    <div class="sidebar">
      <div class="logo">
        <img src="/링키 1차안.png" alt="Linky" class="logo-img" />
        <span class="logo-text"
        @click="setActiveMenu('main')">Linky</span>
      </div>
      
      <nav class="nav-menu">
        <div 
          class="nav-item" 
          :class="{ active: activeMenu === 'dashboard' }"
          @click="setActiveMenu('dashboard')"
        >
          <v-icon class="nav-icon">mdi-view-dashboard</v-icon>
          <span>Dashboard</span>
        </div>
        <div 
          class="nav-item" 
          :class="{ active: activeMenu === 'robot-status' }"
          @click="setActiveMenu('robot-status')"
        >
          <v-icon class="nav-icon">mdi-robot</v-icon>
          <span>Robot Status</span>
        </div>
        <div 
          class="nav-item" 
          :class="{ active: activeMenu === 'likes' }"
          @click="setActiveMenu('likes')"
        >
          <v-icon class="nav-icon">mdi-heart</v-icon>
          <span>Likes</span>
        </div>
        
        
        <div 
          class="nav-item" 
          :class="{ active: activeMenu === 'system-logs' }"
          @click="setActiveMenu('system-logs')"
        >
          <v-icon class="nav-icon">mdi-file-document</v-icon>
          <span>System Logs</span>
        </div>
        
        <div 
          class="nav-item" 
          :class="{ active: activeMenu === 'robot-position' }"
          @click="setActiveMenu('robot-position')"
        >
          <v-icon class="nav-icon">mdi-map-marker</v-icon>
          <span>Robot Position</span>
        </div>
      </nav>
      
      <div class="sidebar-bottom">
        <div class="nav-item" @click="openSettings">
          <v-icon class="nav-icon">mdi-cog</v-icon>
          <span>Settings</span>
        </div>
        <div class="nav-item">
          <v-icon class="nav-icon">mdi-power</v-icon>
          <span>Power</span>
        </div>
      </div>
    </div>
  </template>
  
  <script setup>
  import { ref } from 'vue'
  import { useRouter } from 'vue-router'
  import { defineEmits } from 'vue'
  import SettingsModal from './SettingsModal.vue'
  
  const router = useRouter()
  const emit = defineEmits(['menu-change', 'open-settings'])
  

// 현재 활성 메뉴
const activeMenu = ref('dashboard')

// 환경에 따른 경로 설정
const isDevelopment = import.meta.env.DEV
const prefix = isDevelopment ? '' : '/admin'

// 라우트 매핑
const routes = {
  'dashboard': `${prefix}/dashboard-detail`,
  'robot-status': `${prefix}/robot-status`,
  'likes': `${prefix}/likes`,
  'chat': `${prefix}/chat`,
  'calendar': `${prefix}/calendar`,
  'system-logs': `${prefix}/log`,
  'driving-stream': `${prefix}/webrtc-test`,
  'robot-position': `${prefix}/robot-position`,
  'main': `${prefix}/main`
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

// Settings 모달 열기
const openSettings = () => {
  emit('open-settings')
}
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
    cursor: pointer;
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
  

  
  .nav-icon {
    margin-right: 12px;
    font-size: 18px;
    color: inherit;
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