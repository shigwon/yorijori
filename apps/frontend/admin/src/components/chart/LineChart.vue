<template>
  <div class="line-chart-container">
    <!-- 날짜 선택 영역 -->
    <div class="date-selector">
      <label for="date-picker" class="date-label">날짜 선택:</label>
      <input
        id="date-picker"
        type="date"
        v-model="chartStore.selectedDate"
        @change="fetchChartData"
        class="date-input"
      />
    </div>

    <!-- 로딩 상태 -->
    <div v-if="chartStore.isLoading" class="loading-container">
      <div class="loading-spinner"></div>
      <p class="loading-text">데이터를 불러오는 중...</p>
    </div>

    <!-- 에러 상태 -->
    <div v-else-if="chartStore.error" class="error-container">
      <p class="error-text">{{ chartStore.error }}</p>
      <button @click="fetchChartData" class="retry-button">다시 시도</button>
    </div>

    <!-- 차트 표시 -->
    <div v-else class="chart-area">
      <VueApexCharts
        :options="chartOptions"
        :series="series"
        type="line"
        :height="height"
      />
    </div>
  </div>
</template>

<script setup>
import { computed, onMounted } from 'vue'
import VueApexCharts from 'vue3-apexcharts'
import { useChartStore } from '../../stores/chartStore.js'
import { getHourlyOrderStats } from '../../api/examples.js'

const chartStore = useChartStore()

const props = defineProps({
  colors: {
    type: Array,
    default: () => ['#1e40af']
  },
  height: {
    type: [String, Number],
    default: 300
  }
})

// API에서 데이터 가져오기
const fetchChartData = async () => {
  try {
    chartStore.setLoading(true)
    chartStore.clearError()
    
    const response = await getHourlyOrderStats(chartStore.selectedDate)
    
    if (response.result === 'success' && response.data?.hourlyData) {
      // 시간별 데이터 처리 (0시부터 23시까지 모든 시간 포함)
      const hourlyData = response.data.hourlyData
      
      // 0시부터 23시까지 모든 시간에 대한 데이터 생성
      const allHours = Array.from({ length: 24 }, (_, i) => i)
      const dataMap = new Map(hourlyData.map(item => [item.hour, item.count]))
      
      // 모든 시간에 대해 데이터 생성 (없는 시간은 0으로 설정)
      const chartData = allHours.map(hour => dataMap.get(hour) || 0)
      const chartCategories = allHours.map(hour => `${hour}시`)
      
      chartStore.setHourlyData({
        data: chartData,
        categories: chartCategories
      })
    } else {
      throw new Error('데이터 형식이 올바르지 않습니다.')
    }
  } catch (err) {
    console.error('차트 데이터 로드 오류:', err)
    chartStore.setError(err.message || '데이터 로드에 실패했습니다.')
    
    // 에러 시 기본 데이터 사용 (테스트용 - 0시부터 23시까지)
    const defaultData = Array.from({ length: 24 }, (_, i) => {
      if (i >= 7 && i <= 22) return Math.floor(Math.random() * 30) + 5
      return 0
    })
    const defaultCategories = Array.from({ length: 24 }, (_, i) => `${i}시`)
    
    chartStore.setHourlyData({
      data: defaultData,
      categories: defaultCategories
    })
  } finally {
    chartStore.setLoading(false)
  }
}

// 컴포넌트 마운트 시 데이터 로드
onMounted(() => {
  fetchChartData()
})

const series = computed(() => {
  return [{
    name: '시간별 주문 수',
    data: chartStore.hourlyData.data || []
  }]
})

const chartOptions = computed(() => {
  return {
    chart: {
      type: 'line',
      toolbar: {
        show: false
      },
      background: 'transparent'
    },
    stroke: {
      curve: 'smooth',
      width: 5,
      colors: props.colors
    },
    fill: {
      type: 'gradient',
      gradient: {
        shade: 'dark',
        type: 'vertical',
        shadeIntensity: 0.2,
        gradientToColors: props.colors,
        inverseColors: false,
        opacityFrom: 0.5,
        opacityTo: 0.2,
        stops: [0, 100]
      }
    },
    dataLabels: {
      enabled: false
    },
    xaxis: {
      categories: chartStore.hourlyData.categories || [],
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
          }
        },
        min: 0,
        max: Math.max(...(chartStore.hourlyData.data || [0])) + 10, // 최대값 + 여유분
        tickAmount: 5,
        forceNiceScale: true
      },
    tooltip: {
      theme: 'dark',
      y: {
        formatter: function (val) {
          return val.toFixed(0)
        }
      }
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
    },
    markers: {
      size: 7,
      colors: props.colors,
      strokeColors: '#ffffff',
      strokeWidth: 2,
      hover: {
        size: 9
      }
    }
  }
})
</script>

<style scoped>
.line-chart-container {
  width: 100%;
  height: 100%;
  display: flex;
  flex-direction: column;
}

/* 날짜 선택 영역 */
.date-selector {
  display: flex;
  align-items: center;
  gap: 12px;
  margin-bottom: 16px;
  padding: 12px;
  background-color: #2a2f3e;
  border-radius: 8px;
}

.date-label {
  color: #b6bace;
  font-size: 14px;
  font-weight: 500;
  white-space: nowrap;
}

.date-input {
  background-color: #222738;
  border: 1px solid #374151;
  border-radius: 6px;
  padding: 8px 12px;
  color: #ffffff;
  font-size: 14px;
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

/* 차트 영역 */
.chart-area {
  flex: 1;
  width: 100%;
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