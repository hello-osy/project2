original_min = 1
original_max = 10
new_min = 100
new_max = 50
def normalize(value, original_min, original_max, new_min, new_max):
    original_range = original_max - original_min
    new_range = new_max - new_min
    normalized_value = ((value - original_min) / original_range) * new_range + new_min
    return normalized_value

# 정규화 예제
values = list(range(1, 11))  # 1부터 10까지의 값

normalized_values = [normalize(value, original_min, original_max, new_min, new_max) for value in values]

print(f'Original values: {values}')
print(f'Normalized values: {normalized_values}')
