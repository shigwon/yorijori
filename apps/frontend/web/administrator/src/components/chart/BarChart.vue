<template>
    <div class="bar-chart-container">
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
      <VueApexCharts
        v-else
        :options="chartOptions"
        :series="series"
        type="bar"
        height="100%"
      />
    </div>
  </template>
  
  <script setup>
  import { computed, ref, onMounted } from 'vue'
  import VueApexCharts from 'vue3-apexcharts'
  import { getWeeklyOrderCount } from '../../api/examples.js'

  // 차트 데이터 상태
  const chartData = ref([])
  const chartCategories = ref([])
  const isLoading = ref(true)
  const error = ref(null)

  // API에서 데이터 가져오기
  const fetchChartData = async () => {
    try {
      isLoading.value = true
      error.value = null
      
      const response = await getWeeklyOrderCount()
      
      if (response.result === 'success' && response.data?.dailyData) {
        // 요일 순서대로 정렬 (월요일부터 일요일까지)
        const sortedData = response.data.dailyData.sort((a, b) => a.dayOfWeek - b.dayOfWeek)
        
        // 차트 데이터와 카테고리 추출
        chartData.value = sortedData.map(item => item.count)
        chartCategories.value = sortedData.map(item => item.dayName)
      } else {
        throw new Error('데이터 형식이 올바르지 않습니다.')
      }
    } catch (err) {
      console.error('차트 데이터 로드 오류:', err)
      error.value = err.message
      // 에러 시 기본 데이터 사용
      chartData.value = [0, 0, 0, 0, 0, 0, 0]
      chartCategories.value = ['월', '화', '수', '목', '금', '토', '일']
    } finally {
      isLoading.value = false
    }
  }

  // 컴포넌트 마운트 시 데이터 로드
  onMounted(() => {
    fetchChartData()
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
    background-color: transparent;
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
  </style> 