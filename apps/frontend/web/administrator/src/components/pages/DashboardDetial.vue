<template>
  <div class="charts-container">
    <!-- Orders / Day 차트 -->
    <div class="chart-card">
      <div class="chart-header">
        <h3 class="chart-title">Orders / Day</h3>
      </div>
      <div class="chart-content">
        <LineChart 
          :data="dayChartData"
          :categories="dayChartCategories"
          :colors="dayChartColors"
          height="300"
        />
      </div>
    </div>
    <!-- Orders / Week 차트 -->
    <div class="chart-card">
      <div class="chart-header">
        <h3 class="chart-title">Orders / Week</h3>
      </div>
      <div class="chart-content">
        <BarChart 
          :data="weekChartData"
          :categories="weekChartCategories"
          :colors="weekChartColors"
          :yAxisMax="40"
          :yAxisStep="10"
          height="300"
        />
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref } from 'vue'
import BarChart from '../chart/BarChart.vue'
import LineChart from '../chart/LineChart.vue'

// Orders / Day 차트 데이터 (시간별) - 이미지 기반 데이터 (8개 포인트)
const dayChartData = ref([15, 25, 35, 15, 15, 15, 15, 15])
const dayChartCategories = ref(['3', '6', '9', '12', '15', '18', '21', '24'])
const dayChartColors = ref(['#1e40af'])

// Orders / Week 차트 데이터 (요일별) - 이미지 기반 데이터
const weekChartData = ref([25, 30, 20, 25, 35, 35, 20])
const weekChartCategories = ref(['월', '화', '수', '목', '금', '토', '일'])
const weekChartColors = ref(['#1d4ed8'])
</script>

<style scoped>
.dashboard-detail {
  display: flex;
  height: 100vh;
  background-color: #081028;
  color: #ffffff;
}

.main-content {
  flex: 1;
  padding: 20px;
  overflow-y: auto;
}

.charts-container {
  display: flex;
  flex-direction: column;
  gap: 30px;
  margin-top: 20px;
}

.chart-card {
  background-color: #222738;
  border-radius: 12px;
  padding: 24px;
  box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
  min-height: 400px;
}

.chart-header {
  margin-bottom: 20px;
}

.chart-title {
  color: #d3d3d3;
  font-size: 20px;
  font-weight: bold;
  margin: 0;
}

.chart-content {
  height: 300px;
  width: 100%;
}

/* 반응형 디자인 */
@media (max-width: 1200px) {
  .chart-card {
    min-height: 350px;
  }
}

@media (max-width: 768px) {
  .charts-container {
    gap: 20px;
  }
  .chart-card {
    padding: 16px;
    min-height: 300px;
  }
  .chart-title {
    font-size: 18px;
  }
}

@media (max-width: 480px) {
  .main-content {
    padding: 10px;
  }
  .chart-card {
    padding: 12px;
    min-height: 250px;
  }
  .chart-title {
    font-size: 16px;
  }
}
  </style> 