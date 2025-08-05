<template>
    <div class="bar-chart-container">
      <apexchart
        :options="chartOptions"
        :series="series"
        type="bar"
        height="100%"
      />
    </div>
  </template>
  
  <script setup>
  import { computed } from 'vue'
  import VueApexCharts from 'vue3-apexcharts'
  import { defineProps } from 'vue'
  
  const props = defineProps({
    // 차트 데이터
    data: {
      type: Array,
      default: () => []
    },
    // 카테고리 (X축 라벨)
    categories: {
      type: Array,
      default: () => []
    },
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
      name: 'Data',
      data: props.data
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
        categories: props.categories,
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
  </style> 