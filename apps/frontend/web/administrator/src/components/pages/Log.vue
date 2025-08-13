<template>
  <div class="log-page">
    <!-- 로그 테이블 -->
    <div class="log-table-container">
      <div class="table-header">
        <div class="header-cell checkbox-cell">
          <input type="checkbox" class="select-all-checkbox" @change="toggleSelectAll" />
        </div>
        <div class="header-cell order-cell">
          <v-icon class="header-icon">mdi-clipboard-list</v-icon>
          <span>order</span>
        </div>
        <div class="header-cell date-cell">
          <v-icon class="header-icon">mdi-calendar</v-icon>
          <span>date</span>
        </div>
        <div class="header-cell status-cell">
          <v-icon class="header-icon">mdi-check-circle</v-icon>
          <span>status</span>
          <div class="status-filter">
            <v-select
              v-model="statusFilter"
              :items="filterOptions"
              variant="outlined"
              density="compact"
              hide-details
              class="filter-select"
              @update:model-value="applyFilter"
            ></v-select>
          </div>
        </div>
      </div>

      <div class="table-body">
        <div 
          class="log-row" 
          v-for="(log, index) in filteredLogData" 
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
        </div>
      </div>
    </div>

    <!-- 페이지네이션 -->
    <div class="pagination">
      <button class="pagination-btn" :disabled="currentPage === 1" @click="changePage(currentPage - 1)">
        <v-icon>mdi-chevron-left</v-icon>
      </button>
      <span class="page-info">Page {{ currentPage }} of {{ totalPages }}</span>
      <button class="pagination-btn" :disabled="currentPage === totalPages" @click="changePage(currentPage + 1)">
        <v-icon>mdi-chevron-right</v-icon>
      </button>
    </div>
  </div>
</template>

<script setup>
import { ref, computed, onMounted } from 'vue'
import { getLogs } from '../../api/examples'
// Reactive data
const currentPage = ref(1)
const totalPages = ref(5)
const statusFilter = ref('최근 순')

const fetchLogData = async () => {
  try {
    const result = await getLogs()
    console.log('API 응답:', result)
    console.log('로그 데이터:', result.data)
  } catch (error) {
    console.error('로그 데이터 조회 실패:', error)
  }
}


const filterOptions = [
  { title: '최근 순', value: '최근 순' },
  { title: '오래된 순', value: '오래된 순' }
]

const logData = ref([
  {
    order: '&1',
    date: 'Dec 30, 10:24 AM',
    status: 'End',
    selected: false,
    timestamp: new Date('2024-12-30T10:24:00')
  },
  {
    order: '&2',
    date: 'Dec 30, 10:22 AM',
    status: 'End',
    selected: false,
    timestamp: new Date('2024-12-30T10:22:00')
  },
  {
    order: '&3',
    date: 'Dec 30, 10:20 AM',
    status: 'Running',
    selected: false,
    timestamp: new Date('2024-12-30T10:20:00')
  },
  {
    order: '&4',
    date: 'Dec 30, 10:18 AM',
    status: 'End',
    selected: false,
    timestamp: new Date('2024-12-30T10:18:00')
  },
  {
    order: '&5',
    date: 'Dec 30, 10:16 AM',
    status: 'Error',
    selected: false,
    timestamp: new Date('2024-12-30T10:16:00')
  },
  {
    order: '&6',
    date: 'Dec 30, 10:14 AM',
    status: 'End',
    selected: false,
    timestamp: new Date('2024-12-30T10:14:00')
  },
  {
    order: '&7',
    date: 'Dec 30, 10:12 AM',
    status: 'Running',
    selected: false,
    timestamp: new Date('2024-12-30T10:12:00')
  },
  {
    order: '&8',
    date: 'Dec 30, 10:10 AM',
    status: 'End',
    selected: false,
    timestamp: new Date('2024-12-30T10:10:00')
  },
  {
    order: '&9',
    date: 'Dec 30, 10:08 AM',
    status: 'End',
    selected: false,
    timestamp: new Date('2024-12-30T10:08:00')
  },
  {
    order: '&10',
    date: 'Dec 30, 10:06 AM',
    status: 'Error',
    selected: false,
    timestamp: new Date('2024-12-30T10:06:00')
  }
])

// 필터링된 로그 데이터
const filteredLogData = computed(() => {
  let filtered = [...logData.value]
  
  // 날짜 순으로 정렬
  if (statusFilter.value === '최근 순') {
    filtered.sort((a, b) => b.timestamp - a.timestamp)
  } else if (statusFilter.value === '오래된 순') {
    filtered.sort((a, b) => a.timestamp - b.timestamp)
  }
  
  return filtered
})

// Methods
const toggleSelectAll = (event) => {
  const isChecked = event.target.checked
  logData.value.forEach(log => {
    log.selected = isChecked
  })
}

const updateSelection = () => {
  // 개별 선택 상태 업데이트
  console.log('선택된 로그:', logData.value.filter(log => log.selected))
}

const applyFilter = () => {
  // 필터 적용 시 추가 로직이 필요한 경우
  console.log('필터 적용:', statusFilter.value)
}

const changePage = (page) => {
  if (page >= 1 && page <= totalPages.value) {
    currentPage.value = page
  }
}

// 컴포넌트 마운트 시 로그 데이터 조회
onMounted(() => {
  fetchLogData()
})
</script>

<style scoped>
.log-page {
  padding-top: 20px;
  height: 100%;
  background-color: #081028;
  color: #ffffff;
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
  grid-template-columns: 50px 1fr 1fr 1fr;
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
  color: #3a57e8;
}

.status-filter {
  margin-left: auto;
}

.filter-select {
  min-width: 120px;
}

.table-body {
  max-height: 500px;
  overflow-y: auto;
}

.log-row {
  display: grid;
  grid-template-columns: 50px 1fr 1fr 1fr;
  gap: 16px;
  padding: 16px 20px;
  border-bottom: 1px solid #2a2f3e;
  transition: all 0.3s ease;
  align-items: center;
}

.log-row:hover {
  background-color: #2a2f3e;
}

.log-row.selected {
  background-color: rgba(58, 87, 232, 0.2);
  border-left: 4px solid #3a57e8;
  transform: translateX(4px);
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
  cursor: pointer;
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
  display: flex;
  align-items: center;
  justify-content: center;
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

/* Vuetify 컴포넌트 스타일 오버라이드 */
:deep(.v-select .v-field) {
  background-color: #2a2f3e;
  border-color: #3a3f4e;
}

:deep(.v-select .v-field__input) {
  color: #ffffff;
  font-size: 12px;
}

:deep(.v-select .v-field__append-inner) {
  color: #8a92a6;
}

:deep(.v-select .v-field:hover) {
  border-color: #3a57e8;
}

/* 반응형 디자인 */
@media (max-width: 768px) {
  .table-header, .log-row {
    grid-template-columns: 40px 1fr 1fr 1fr;
    gap: 12px;
    padding: 12px 16px;
  }
  
  .header-cell, .row-cell {
    font-size: 12px;
  }
  
  .filter-select {
    min-width: 100px;
  }
}

@media (max-width: 480px) {
  .log-page {
    padding: 10px;
  }
  
  .table-header, .log-row {
    grid-template-columns: 30px 1fr 1fr 1fr;
    gap: 8px;
    padding: 8px 12px;
  }
  
  .header-cell, .row-cell {
    font-size: 11px;
  }
  
  .filter-select {
    min-width: 80px;
  }
}
</style> 