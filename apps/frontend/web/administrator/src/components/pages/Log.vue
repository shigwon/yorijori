<template>
    <div class="log-page">
      <!-- 헤더 -->
      <div class="log-header">
        <div class="header-left">
          <div class="title-section">
            <div class="red-dot"></div>
            <h1 class="page-title">Log</h1>
          </div>
        </div>
        <div class="header-right">
          <div class="filter-controls">
            <select class="filter-select">
              <option>All Status</option>
              <option>End</option>
              <option>Running</option>
              <option>Error</option>
            </select>
            <button class="refresh-btn">
              <i class="refresh-icon">🔄</i>
            </button>
          </div>
        </div>
      </div>
  
      <!-- 로그 테이블 -->
      <div class="log-table-container">
        <div class="table-header">
          <div class="header-cell checkbox-cell">
            <input type="checkbox" class="select-all-checkbox" @change="toggleSelectAll" />
          </div>
          <div class="header-cell order-cell">
            <i class="header-icon">📋</i>
            <span>order</span>
          </div>
          <div class="header-cell date-cell">
            <i class="header-icon">📅</i>
            <span>date</span>
          </div>
          <div class="header-cell status-cell">
            <i class="header-icon">✅</i>
            <span>status</span>
          </div>
          <div class="header-cell action-cell">
            <span>action</span>
          </div>
        </div>
  
        <div class="table-body">
          <div 
            class="log-row" 
            v-for="(log, index) in logData" 
            :key="index"
            :class="{ 'selected': log.selected }"
          >
            <div class="row-cell checkbox-cell">
              <input 
                type="checkbox" 
                class="log-checkbox" 
                v-model="log.selected"
                @change="updateSelection"
              />
            </div>
            <div class="row-cell order-cell">
              <span class="order-number">{{ log.order }}</span>
            </div>
            <div class="row-cell date-cell">
              <span class="date-text">{{ log.date }}</span>
            </div>
            <div class="row-cell status-cell">
              <div class="status-badge" :class="log.status.toLowerCase()">
                <div class="status-dot"></div>
                <span>{{ log.status }}</span>
              </div>
            </div>
            <div class="row-cell action-cell">
              <button class="action-btn" @click="viewDetails(log)">
                <i class="action-icon">👁️</i>
              </button>
              <button class="action-btn" @click="deleteLog(log)">
                <i class="action-icon">🗑️</i>
              </button>
            </div>
          </div>
        </div>
      </div>
  
      <!-- 페이지네이션 -->
      <div class="pagination">
        <button class="pagination-btn" :disabled="currentPage === 1" @click="changePage(currentPage - 1)">
          <i class="pagination-icon">←</i>
        </button>
        <span class="page-info">Page {{ currentPage }} of {{ totalPages }}</span>
        <button class="pagination-btn" :disabled="currentPage === totalPages" @click="changePage(currentPage + 1)">
          <i class="pagination-icon">→</i>
        </button>
      </div>
    </div>
  </template>
  
  <script setup>
  import { ref } from 'vue'
  
  // Reactive data
  const currentPage = ref(1)
  const totalPages = ref(5)
  const logData = ref([
    {
      order: '&1',
      date: 'Dec 30, 10:24 AM',
      status: 'End',
      selected: false
    },
    {
      order: '&2',
      date: 'Dec 30, 10:22 AM',
      status: 'End',
      selected: false
    },
    {
      order: '&3',
      date: 'Dec 30, 10:20 AM',
      status: 'Running',
      selected: false
    },
    {
      order: '&4',
      date: 'Dec 30, 10:18 AM',
      status: 'End',
      selected: false
    },
    {
      order: '&5',
      date: 'Dec 30, 10:16 AM',
      status: 'Error',
      selected: false
    },
    {
      order: '&6',
      date: 'Dec 30, 10:14 AM',
      status: 'End',
      selected: false
    },
    {
      order: '&7',
      date: 'Dec 30, 10:12 AM',
      status: 'Running',
      selected: false
    },
    {
      order: '&8',
      date: 'Dec 30, 10:10 AM',
      status: 'End',
      selected: false
    },
    {
      order: '&9',
      date: 'Dec 30, 10:08 AM',
      status: 'End',
      selected: false
    },
    {
      order: '&10',
      date: 'Dec 30, 10:06 AM',
      status: 'Error',
      selected: false
    }
  ])
  
  // Methods
  const toggleSelectAll = (event) => {
    const isChecked = event.target.checked
    logData.value.forEach(log => {
      log.selected = isChecked
    })
  }
  
  const updateSelection = () => {
    // 개별 선택 상태 업데이트
  }
  
  const viewDetails = (log) => {
    console.log('상세 보기:', log)
    // 상세 모달 또는 페이지로 이동
  }
  
  const deleteLog = (log) => {
    console.log('로그 삭제:', log)
    // 삭제 확인 모달 표시
  }
  
  const changePage = (page) => {
    if (page >= 1 && page <= totalPages.value) {
      currentPage.value = page
    }
  }
  </script>
  
  <style scoped>
  .log-page {
    padding: 20px;
    height: 100%;
    background-color: #081028;
    color: #ffffff;
  }
  
  /* 헤더 스타일 */
  .log-header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-bottom: 30px;
    padding-bottom: 20px;
    border-bottom: 1px solid #2a2f3e;
  }
  
  .header-left {
    display: flex;
    align-items: center;
  }
  
  .title-section {
    display: flex;
    align-items: center;
    gap: 12px;
  }
  
  .red-dot {
    width: 12px;
    height: 12px;
    background-color: #ff4757;
    border-radius: 50%;
  }
  
  .page-title {
    font-size: 28px;
    font-weight: bold;
    margin: 0;
  }
  
  .header-right {
    display: flex;
    align-items: center;
    gap: 16px;
  }
  
  .filter-controls {
    display: flex;
    align-items: center;
    gap: 12px;
  }
  
  .filter-select {
    padding: 8px 12px;
    background-color: #222738;
    border: 1px solid #2a2f3e;
    border-radius: 6px;
    color: #ffffff;
    font-size: 14px;
  }
  
  .refresh-btn {
    background: none;
    border: none;
    color: #3a57e8;
    font-size: 18px;
    cursor: pointer;
    padding: 8px;
    border-radius: 4px;
    transition: background-color 0.3s ease;
  }
  
  .refresh-btn:hover {
    background-color: rgba(58, 87, 232, 0.1);
  }
  
  /* 테이블 스타일 */
  .log-table-container {
    background-color: #222738;
    border-radius: 12px;
    overflow: hidden;
    box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
  }
  
  .table-header {
    display: grid;
    grid-template-columns: 50px 1fr 1fr 1fr 120px;
    gap: 16px;
    padding: 16px 20px;
    background-color: #1a1f2e;
    border-bottom: 1px solid #2a2f3e;
    font-weight: bold;
    font-size: 14px;
  }
  
  .header-cell {
    display: flex;
    align-items: center;
    gap: 8px;
    color: #b6bace;
  }
  
  .header-icon {
    font-size: 16px;
  }
  
  .table-body {
    max-height: 500px;
    overflow-y: auto;
  }
  
  .log-row {
    display: grid;
    grid-template-columns: 50px 1fr 1fr 1fr 120px;
    gap: 16px;
    padding: 16px 20px;
    border-bottom: 1px solid #2a2f3e;
    transition: background-color 0.3s ease;
    align-items: center;
  }
  
  .log-row:hover {
    background-color: #2a2f3e;
  }
  
  .log-row.selected {
    background-color: rgba(58, 87, 232, 0.1);
  }
  
  .row-cell {
    display: flex;
    align-items: center;
    font-size: 14px;
  }
  
  .checkbox-cell {
    justify-content: center;
  }
  
  .log-checkbox, .select-all-checkbox {
    width: 18px;
    height: 18px;
    accent-color: #3a57e8;
  }
  
  .order-number {
    font-weight: 500;
    color: #3a57e8;
  }
  
  .date-text {
    color: #8a92a6;
  }
  
  .status-badge {
    display: flex;
    align-items: center;
    gap: 6px;
    padding: 4px 8px;
    border-radius: 4px;
    font-size: 12px;
    font-weight: 500;
    width: fit-content;
  }
  
  .status-badge.end {
    background-color: #13c572;
    color: white;
  }
  
  .status-badge.running {
    background-color: #3a57e8;
    color: white;
  }
  
  .status-badge.error {
    background-color: #ff4757;
    color: white;
  }
  
  .status-dot {
    width: 6px;
    height: 6px;
    border-radius: 50%;
    background-color: currentColor;
  }
  
  .action-cell {
    display: flex;
    gap: 8px;
  }
  
  .action-btn {
    background: none;
    border: none;
    color: #8a92a6;
    cursor: pointer;
    padding: 4px;
    border-radius: 4px;
    transition: all 0.3s ease;
  }
  
  .action-btn:hover {
    background-color: rgba(255, 255, 255, 0.1);
    color: #ffffff;
  }
  
  .action-icon {
    font-size: 14px;
  }
  
  /* 페이지네이션 */
  .pagination {
    display: flex;
    justify-content: center;
    align-items: center;
    gap: 16px;
    margin-top: 30px;
    padding-top: 20px;
    border-top: 1px solid #2a2f3e;
  }
  
  .pagination-btn {
    background: none;
    border: 1px solid #2a2f3e;
    color: #b6bace;
    padding: 8px 12px;
    border-radius: 6px;
    cursor: pointer;
    transition: all 0.3s ease;
  }
  
  .pagination-btn:hover:not(:disabled) {
    background-color: #3a57e8;
    color: white;
    border-color: #3a57e8;
  }
  
  .pagination-btn:disabled {
    opacity: 0.5;
    cursor: not-allowed;
  }
  
  .page-info {
    color: #8a92a6;
    font-size: 14px;
  }
  
  /* 반응형 디자인 */
  @media (max-width: 768px) {
    .log-header {
      flex-direction: column;
      gap: 16px;
      align-items: flex-start;
    }
    
    .table-header, .log-row {
      grid-template-columns: 40px 1fr 1fr 1fr 80px;
      gap: 12px;
      padding: 12px 16px;
    }
    
    .header-cell, .row-cell {
      font-size: 12px;
    }
    
    .page-title {
      font-size: 24px;
    }
  }
  
  @media (max-width: 480px) {
    .log-page {
      padding: 10px;
    }
    
    .table-header, .log-row {
      grid-template-columns: 30px 1fr 1fr 1fr 60px;
      gap: 8px;
      padding: 8px 12px;
    }
    
    .header-cell, .row-cell {
      font-size: 11px;
    }
    
    .page-title {
      font-size: 20px;
    }
    
    .filter-controls {
      flex-direction: column;
      gap: 8px;
    }
  }
  </style> 