<template>
  <div class="charts-container">
    <!-- Orders / Day 차트 -->
    <div class="chart-card">
      <div class="chart-header">
        <h3 class="chart-title">시간별 주문 통계</h3>
      </div>
      <div class="chart-content">
        <LineChart 
          :colors="dayChartColors"
          height="100%"
        />
      </div>
    </div>
    <!-- Orders / Week 차트 -->
    <div class="chart-card">
      <div class="chart-header">
        <h3 class="chart-title">주간 주문 통계</h3>
      </div>
      <div class="chart-content">
        <BarChart 
          :colors="weekChartColors"
          :yAxisMax="40"
          :yAxisStep="10"
          height="100%"
        />
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref } from 'vue'
import BarChart from '../chart/BarChart.vue'
import LineChart from '../chart/LineChart.vue'

// 차트 색상 설정
const dayChartColors = ref(['#1e40af'])
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
  gap: 20px;
  margin-top: 20px;
  height: calc(100vh - 100px);
  overflow: hidden;
}

.chart-card {
  background-color: #222738;
  border-radius: 12px;
  padding: 24px;
  box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
  flex: 1;
  display: flex;
  flex-direction: column;
  min-height: 0;
  overflow: hidden;
}

.chart-header {
  margin-bottom: 20px;
  flex-shrink: 0;
}

.chart-title {
  color: #d3d3d3;
  font-size: 20px;
  font-weight: bold;
  margin: 0;
}

.chart-content {
  flex: 1;
  width: 100%;
  min-height: 0;
  overflow: hidden;
}

/* 반응형 디자인 */
@media (max-width: 1200px) {
  .charts-container {
    height: calc(100vh - 80px);
  }
}

@media (max-width: 768px) {
  .charts-container {
    gap: 20px;
    height: calc(100vh - 60px);
  }
  .chart-card {
    padding: 16px;
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
  }
  .chart-title {
    font-size: 16px;
  }
}
  </style> 