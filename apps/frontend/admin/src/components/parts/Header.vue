<template>
    <header class="header">
      <div class="header-left">
        <h1>{{ title }}</h1>
      </div>
      <div class="header-right">
        <div class="notification" @click="handleNotificationClick">
          <v-icon class="notification-icon">mdi-bell</v-icon>
          <span class="notification-badge">{{ notificationCount }}</span>
        </div>
        <div class="user-profile" @click="handleProfileClick">
          <img :src="userAvatar" alt="Profile" class="profile-img" />
          <div class="user-info">
            <span class="user-name">{{ userName }}</span>
            <span class="user-role">{{ userRole }}</span>
          </div>
        </div>
      </div>
    </header>
  </template>
  
  <script setup>
  import { defineProps, defineEmits } from 'vue'
  import NotificationModal from './NotificationModal.vue'
  
  const props = defineProps({
    title: {
      type: String,
      default: 'Administrator'
    },
    notificationCount: {
      type: Number,
      default: 6
    },
    userName: {
      type: String,
      default: 'Moni Roy'
    },
    userRole: {
      type: String,
      default: 'Admin'
    },
    userAvatar: {
      type: String,
      default: '/피카츄.png'
    }
  })
  
  const emit = defineEmits(['notification-click', 'profile-click', 'notification-count-changed'])
  
  const handleNotificationClick = () => {
    emit('notification-click')
  }
  
  const handleProfileClick = () => {
    emit('profile-click')
  }
  
  const handleNotificationCountChanged = (count) => {
    emit('notification-count-changed', count)
  }
  </script>
  
  <style scoped>
  .header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    
    padding-bottom: 20px;
    border-bottom: 1px solid #2a2f3e;
  }
  
  .header h1 {
    font-size: 28px;
    font-weight: bold;
    margin: 0;
  }
  
  .header-right {
    display: flex;
    align-items: center;
    gap: 20px;
  }
  
  .notification {
    position: relative;
    cursor: pointer;
    transition: transform 0.2s ease;
  }
  
  .notification:hover {
    transform: scale(1.1);
  }
  
  .notification-icon {
    font-size: 20px;
    color: inherit;
  }
  
  .notification-badge {
    position: absolute;
    top: -8px;
    right: -8px;
    background-color: #ff4757;
    color: white;
    border-radius: 50%;
    width: 20px;
    height: 20px;
    display: flex;
    align-items: center;
    justify-content: center;
    font-size: 12px;
    animation: pulse 2s infinite;
  }
  
  @keyframes pulse {
    0% {
      transform: scale(1);
    }
    50% {
      transform: scale(1.1);
    }
    100% {
      transform: scale(1);
    }
  }
  
  .user-profile {
    display: flex;
    align-items: center;
    gap: 12px;
    cursor: pointer;
    padding: 8px 12px;
    border-radius: 8px;
    transition: background-color 0.3s ease;
  }
  
  .user-profile:hover {
    background-color: #2a2f3e;
  }
  
  .profile-img {
    width: 40px;
    height: 40px;
    border-radius: 50%;
    background-color: #3a57e8;
    transition: transform 0.2s ease;
  }
  
  .user-profile:hover .profile-img {
    transform: scale(1.05);
  }
  
  .user-info {
    display: flex;
    flex-direction: column;
  }
  
  .user-name {
    font-weight: bold;
    font-size: 14px;
  }
  
  .user-role {
    font-size: 12px;
    color: #8a92a6;
  }
  
  /* 반응형 디자인 */
  @media (max-width: 768px) {
    .header h1 {
      font-size: 24px;
    }
    
    .header-right {
      gap: 15px;
    }
    
    .user-profile {
      padding: 6px 8px;
    }
    
    .profile-img {
      width: 35px;
      height: 35px;
    }
    
    .user-name {
      font-size: 13px;
    }
    
    .user-role {
      font-size: 11px;
    }
  }
  
  @media (max-width: 480px) {
    .header {
      flex-direction: column;
      gap: 15px;
      align-items: flex-start;
    }
    
    .header h1 {
      font-size: 20px;
    }
    
    .header-right {
      width: 100%;
      justify-content: space-between;
    }
  }
  </style> 