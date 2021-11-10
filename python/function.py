def spy_game(nums):
	code = []
	for num in nums:
		if num == 0:
			code.append(0)
		elif num == 7:
			code.append(7)
			print(code)
			break

	return code == [0,0,7]

print(spy_game([1,2,3,0,0,7,9,7]))