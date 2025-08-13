<template>
    <div class="bar-chart-container">
      <!-- 날짜 선택 영역 -->
      <div class="date-selector">
        <label for="week-picker" class="date-label">주간 선택:</label>
        <input
          id="week-picker"
          type="date"
          v-model="selectedWeekStart"
          @change="fetchChartData"
          class="date-input"
        />
      </div>

      <!-- 로딩 상태 -->
      <div v-if="isLoading" class="loading-container">
        <div class="loading-spinner"></div>
        <p class="loading-text">데이터를 불러오는 중...</p>
      </div>
      
      <!-- 에러 상태 -->
      <div v-else-if="error" class="error-container">
        <p class="error-text">{{ error }}</p>
        <button @click="fetchChartData" class="retry-button">다시 시도</button>
      </div>
      
      <!-- 차트 표시 -->
      <div v-else class="chart-area">
        <VueApexCharts
          :options="chartOptions"
          :series="series"
          type="bar"
          :height="height"
        />
      </div>
    </div>
  </template>
  
  <script setup>
  import { computed, ref, onMounted } from 'vue'
  import VueApexCharts from 'vue3-apexcharts'
  import { getWeeklyOrderStats } from '../../api/examples.js'

  // 차트 데이터 상태
  const chartData = ref([])
  const chartCategories = ref([])
  const isLoading = ref(true)
  const error = ref(null)
  const selectedWeekStart = ref('')

  // 현재 날짜 기준으로 주간 데이터 가져오기
  const getCurrentWeekStartDate = () => {
    const today = new Date()
    const dayOfWeek = today.getDay() // 0: 일요일, 1: 월요일, ..., 6: 토요일
    const daysToSubtract = dayOfWeek === 0 ? 6 : dayOfWeek - 1 // 월요일을 시작으로 설정
    
    const monday = new Date(today)
    monday.setDate(today.getDate() - daysToSubtract)
    
    // YYYY-MM-DD 형식으로 반환
    return monday.toISOString().split('T')[0]
  }

  // API에서 데이터 가져오기
  const fetchChartData = async () => {
    try {
      isLoading.value = true
      error.value = null
      
             // 선택된 날짜 또는 현재 주의 월요일 날짜 사용
       const startDate = selectedWeekStart.value || getCurrentWeekStartDate()
       console.log('주간 차트 시작 날짜:', startDate)
      
             console.log('BarChart: getWeeklyOrderStats 호출 시작')
       const response = await getWeeklyOrderStats(startDate)
       console.log('BarChart: getWeeklyOrderStats 응답 받음')
       
       // API 응답 디버깅
      //  console.log('API 응답 전체:', response)
      //  console.log('response.result:', response.result)
      //  console.log('response.data:', response.data)
      //  console.log('response.data?.dailyData:', response.data?.dailyData)
      //  console.log('Array.isArray(response.data?.dailyData):', Array.isArray(response.data?.dailyData))
       
       // 다양한 응답 구조 처리
       let dailyData = null
       
       // 구조 1: response.data.dailyData
       if (response.data?.dailyData && Array.isArray(response.data.dailyData)) {
         dailyData = response.data.dailyData
         console.log('구조 1 사용: response.data.dailyData')
       }
       // 구조 2: response.dailyData (직접 접근)
       else if (response.dailyData && Array.isArray(response.dailyData)) {
         dailyData = response.dailyData
         console.log('구조 2 사용: response.dailyData')
       }
       // 구조 3: response.data가 배열인 경우
       else if (Array.isArray(response.data)) {
         dailyData = response.data
         console.log('구조 3 사용: response.data (배열)')
       }
       // 구조 4: response가 배열인 경우
       else if (Array.isArray(response)) {
         dailyData = response
         console.log('구조 4 사용: response (배열)')
       }
       
       if (dailyData && dailyData.length > 0) {
         // 요일 순서대로 정렬 (월요일부터 일요일까지)
         const sortedData = dailyData.sort((a, b) => (a.dayOfWeek || 0) - (b.dayOfWeek || 0))
         
         console.log('정렬된 데이터:', sortedData)
         
         // 차트 데이터와 카테고리 추출
         chartData.value = sortedData.map(item => item.count || 0)
         chartCategories.value = sortedData.map(item => item.dayName || `요일${item.dayOfWeek || ''}`)
         
         console.log('차트 데이터:', chartData.value)
         console.log('차트 카테고리:', chartCategories.value)
       } else {
         console.error('데이터 검증 실패:')
         console.error('- response:', response)
         console.error('- response.data:', response.data)
         console.error('- response.data?.dailyData:', response.data?.dailyData)
         throw new Error('데이터가 비어있거나 형식이 올바르지 않습니다.')
       }
         } catch (err) {
       console.error('BarChart 차트 데이터 로드 오류:', err)
       error.value = err.message
       
       // 에러 시 기본 데이터 사용 (테스트용)
       chartData.value = [12, 19, 15, 25, 22, 30, 18]
       chartCategories.value = ['월', '화', '수', '목', '금', '토', '일']
       
       console.log('BarChart: 에러 시 기본 데이터 설정 완료')
       console.log('차트 데이터:', chartData.value)
       console.log('차트 카테고리:', chartCategories.value)
     } finally {
       isLoading.value = false
     }
  }

  // 컴포넌트 마운트 시 데이터 로드
  onMounted(async () => {
    // 초기 날짜 설정
    selectedWeekStart.value = getCurrentWeekStartDate()
    await fetchChartData()
  })

  const props = defineProps({
    // 색상 배열
    colors: {
      type: Array,
      default: () => ['#1d4ed8']
    },
    // Y축 최대값
    yAxisMax: {
      type: Number,
      default: 160
    },
    // Y축 단위
    yAxisStep: {
      type: Number,
      default: 20
    },
    // 차트 높이
    height: {
      type: [String, Number],
      default: 250
    }
  })
  
  const series = computed(() => {
    return [{
      name: '주문 수',
      data: chartData.value
    }]
  })
  
  const chartOptions = computed(() => {
    return {
      chart: {
        type: 'bar',
        toolbar: {
          show: false
        },
        background: 'transparent'
      },
      plotOptions: {
        bar: {
          borderRadius: 4,
          columnWidth: '30%'
        }
      },
      dataLabels: {
        enabled: false
      },
      stroke: {
        show: true,
        width: 2,
        colors: ['transparent']
      },
      xaxis: {
        categories: chartCategories.value,
        labels: {
          style: {
            colors: '#8a92a6',
            fontSize: '12px'
          }
        },
        axisBorder: {
          show: false
        },
        axisTicks: {
          show: false
        }
      },
      yaxis: {
        labels: {
          style: {
            colors: '#8a92a6',
            fontSize: '12px'
          },
          formatter: function (val) {
            return val.toFixed(0)
          },
          offsetX: -10
        },
        min: 0,
        max: props.yAxisMax,
        tickAmount: props.yAxisStep,
        forceNiceScale: true
      },
      fill: {
        opacity: 1,
        colors: props.colors
      },
      tooltip: {
        y: {
          formatter: function (val) {
            return val.toFixed(0)
          }
        },
        theme: 'dark'
      },
      grid: {
        borderColor: '#374151',
        strokeDashArray: 5,
        xaxis: {
          lines: {
            show: false
          }
        },
        yaxis: {
          lines: {
            show: true,
            color: '#4b5563'
          }
        }
      },
      theme: {
        mode: 'dark'
      }
    }
  })
  </script>
  
  <style scoped>
  .bar-chart-container {
    width: 100%;
    height: 100%;
    display: flex;
    flex-direction: column;
    min-height: 0;
  }
  
  .chart-area {
    flex: 1;
    width: 100%;
    min-height: 0;
    overflow: hidden;
  }

  /* 날짜 선택 영역 */
  .date-selector {
    display: flex;
    align-items: center;
    gap: 8px;
    margin-bottom: 8px;
    padding: 8px;
    background-color: #2a2f3e;
    border-radius: 6px;
    flex-shrink: 0;
  }

  .date-label {
    color: #b6bace;
    font-size: 12px;
    font-weight: 500;
    white-space: nowrap;
  }

  .date-input {
    background-color: #222738;
    border: 1px solid #374151;
    border-radius: 4px;
    padding: 6px 8px;
    color: #ffffff;
    font-size: 12px;
    outline: none;
    transition: border-color 0.3s ease;
  }

  .date-input:focus {
    border-color: #3a57e8;
  }

  .date-input::-webkit-calendar-picker-indicator {
    filter: invert(1);
    cursor: pointer;
  }
  
  /* ApexCharts 컨테이너 스타일링 */
  .bar-chart-container :deep(.apexcharts-canvas) {
    background-color: transparent !important;
  }
  
  .bar-chart-container :deep(.apexcharts-svg) {
    background-color: transparent !important;
  }
  
  /* 로딩 상태 스타일 */
  .loading-container {
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    height: 100%;
    gap: 16px;
  }
  
  .loading-spinner {
    width: 40px;
    height: 40px;
    border: 4px solid #374151;
    border-top: 4px solid #3a57e8;
    border-radius: 50%;
    animation: spin 1s linear infinite;
  }
  
  @keyframes spin {
    0% { transform: rotate(0deg); }
    100% { transform: rotate(360deg); }
  }
  
  .loading-text {
    color: #8a92a6;
    font-size: 14px;
    margin: 0;
  }
  
  /* 에러 상태 스타일 */
  .error-container {
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    height: 100%;
    gap: 16px;
  }
  
  .error-text {
    color: #ef4444;
    font-size: 14px;
    text-align: center;
    margin: 0;
  }
  
  .retry-button {
    background-color: #3a57e8;
    color: white;
    border: none;
    padding: 8px 16px;
    border-radius: 6px;
    font-size: 12px;
    cursor: pointer;
    transition: background-color 0.3s ease;
  }
  
  .retry-button:hover {
    background-color: #4a67f8;
  }

  /* 반응형 디자인 */
  @media (max-width: 768px) {
    .date-selector {
      flex-direction: column;
      align-items: flex-start;
      gap: 8px;
    }
    
    .date-label {
      font-size: 12px;
    }
    
    .date-input {
      width: 100%;
      font-size: 12px;
    }
  }
  </style> 